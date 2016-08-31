#include <string.h>

#include "kl17.h"
#include "usbphy.h"
#include "usbmac.h"
#include "memio.h"

extern void (*portbFastISR)(void);
static struct USBPHY *current_usb_phy;

int usbPhyInitialized(struct USBPHY *phy) {
  if (!phy)
    return 0;

  return phy->initialized;
}

void usbCaptureI(struct USBPHY *phy) {

  uint8_t *samples;
  int ret;

  samples = (uint8_t *)phy->read_queue[phy->read_queue_head];

  ret = usbPhyReadI(phy, samples);
  if (ret <= 0)
    goto out;

  /* Save the byte counter for later inspection */
  samples[11] = ret;

  if (samples[0] == USB_PID_IN) {
    if (!phy->queued_size) {
      uint8_t pkt[] = {USB_PID_NAK};
      usbPhyWriteI(phy, pkt, sizeof(pkt));
      goto out;
    }

    phy->read_queue_head++;
    usbPhyWriteI(phy, phy->queued_data, phy->queued_size);
    goto out;
  }
  else if (samples[0] == USB_PID_SETUP) {
    phy->read_queue_head++;
    goto out;
  }
  else if (samples[0] == USB_PID_OUT) {
    phy->read_queue_head++;
    goto out;
  }
  else if (samples[0] == USB_PID_ACK) {
    /* Allow the next byte to be sent */
    phy->queued_size = 0;
    usbMacTransferSuccess(phy->mac);
    phy->read_queue_head++;
    goto out;
  }

  else if ((samples[0] == USB_PID_DATA0) || (samples[0] == USB_PID_DATA1)) {
    phy->read_queue_head++;
    uint8_t pkt[] = {USB_PID_ACK};
    usbPhyWriteI(phy, pkt, sizeof(pkt));
    goto out;
  }

out:

  phy->read_queue_head &= PHY_READ_QUEUE_MASK;
  return;
}

int usbPhyWritePrepare(struct USBPHY *phy, const void *buffer, int size) {

  phy->queued_data = buffer;
  __DMB();
  phy->queued_size = size;
  return 0;
}

struct USBPHY *usbPhyTestPhy(void) {

  return NULL;
}

void usbPhyWorker(struct USBPHY *phy) {
  while (phy->read_queue_tail != phy->read_queue_head) {
    uint8_t *in_ptr = (uint8_t *)phy->read_queue[phy->read_queue_tail];
    int count = in_ptr[11];

    usbMacProcess(phy->mac, in_ptr, count);

    // Advance to the next packet
    phy->read_queue_tail++;
    phy->read_queue_tail &= PHY_READ_QUEUE_MASK;
  }
  return;
}

#if defined(_CHIBIOS_RT_)
static THD_FUNCTION(usb_worker_thread, arg) {

  struct USBPHY *phy = arg;

  chRegSetThreadName("USB poll thread");
  while (1) {
    osalSysLock();
    (void) osalThreadSuspendS(&phy->thread);
    osalSysUnlock();

    usbPhyWorker(phy);
  }

  return;
}
#endif

#if defined(_CHIBIOS_RT_)  
static void usb_phy_fast_isr(void)
{
  /* Note: We can't use ANY ChibiOS threads here.
   * This thread may interrupt the SysTick handler, which would cause
   * Major Problems if we called OSAL_IRQ_PROLOGUE().
   * To get around this, we simply examine the buffer every time SysTick
   * exits (via CH_CFG_SYSTEM_TICK_HOOK), and wake up the thread from
   * within the SysTick context.
   * That way, this function is free to preempt EVERYTHING without
   * interfering with the timing of the system.
   */
  struct USBPHY *phy = current_usb_phy;
  usbCaptureI(phy);

  /* Clear all pending interrupts on this port. */
  PORTA->ISFR = 0xFFFFFFFF;
}
#endif

#if defined(_CHIBIOS_RT_)
/* Called when the PHY is disconnected, to prevent ChibiOS from
 * overwriting areas of memory that haven't been initialized yet.
 */
static void usb_phy_fast_isr_disabled(void) {

  PORTA->ISFR = 0xFFFFFFFF;
}
#endif

void usbPhyInit(struct USBPHY *phy, struct USBMAC *mac) {

  current_usb_phy = phy;
#if defined(_CHIBIOS_RT_)
  chEvtObjectInit(&phy->data_available);
  chThdCreateStatic(phy->waThread, sizeof(phy->waThread),
                    HIGHPRIO+1, usb_worker_thread, phy);
  portbFastISR = usb_phy_fast_isr_disabled;
#endif
  phy->mac = mac;
  usbMacSetPhy(mac, phy);
  phy->initialized = 1;
  usbPhyDetach(phy);
}

#if defined(_CHIBIOS_RT_)
void usbPhyDrainIfNecessary(void) {
  struct USBPHY *phy = current_usb_phy;

  if (phy->read_queue_tail != phy->read_queue_head)
    osalThreadResumeI(&phy->thread, MSG_OK);
}
#endif

void usbPhyDetach(struct USBPHY *phy) {

  /* Set both lines to 0 (clear both D+ and D-) to simulate unplug. */
  writel(phy->usbdpMask, phy->usbdpCAddr);
  writel(phy->usbdnMask, phy->usbdnCAddr);

  /* Set both lines to output */
  writel(readl(phy->usbdpDAddr) | phy->usbdpMask, phy->usbdpDAddr);
  writel(readl(phy->usbdnDAddr) | phy->usbdnMask, phy->usbdnDAddr);

#if defined(_CHIBIOS_RT_)
  portbFastISR = usb_phy_fast_isr_disabled;
#endif
}

void usbPhyAttach(struct USBPHY *phy) {

  current_usb_phy = phy;

#if defined(_CHIBIOS_RT_)
  /* Hook our GPIO IRQ */
  portbFastISR = usb_phy_fast_isr;
#endif

  /* Set both lines to input */
  writel(readl(phy->usbdpDAddr) & ~phy->usbdpMask, phy->usbdpDAddr);
  writel(readl(phy->usbdnDAddr) & ~phy->usbdnMask, phy->usbdnDAddr);
}
