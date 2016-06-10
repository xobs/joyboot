
#include "kl17.h"

/**
 * @brief   KL1x clocks and PLL initialization.
 * @note    All the involved constants come from the file @p board.h.
 * @note    This function should be invoked just after the system reset.
 *
 * @special
 */
void kl1x_clock_init(void) {
  /* Disable COP watchdog */
  SIM->COPC = 0;

  /* Enable PORTA */
  SIM->SCGC5 |= SIM_SCGC5_PORTA;

  /* --- MCG mode: FEI (default out of reset) ---
     f_MCGOUTCLK = f_int * F
     F is the FLL factor selected by C4[DRST_DRS] and C4[DMX32] bits.
     Typical f_MCGOUTCLK = 21 MHz immediately after reset.
     C4[DMX32]=0 and C4[DRST_DRS]=00  =>  FLL factor=640.
     C3[SCTRIM] and C4[SCFTRIM] factory trim values apply to f_int. */

  /* System oscillator drives 32 kHz clock (OSC32KSEL=0) */
  SIM->SOPT1 &= ~SIM_SOPT1_OSC32KSEL_MASK;

  /*
   * PLL Enabled External (PEE) MCG Mode
   * 48 MHz core, 24 MHz bus - using 32 MHz crystal with PLL.
   * f_MCGOUTCLK = (OSCCLK / PLL_R) * M
   *             =  32 MHz / 8 * 24 = 96 MHz
   *  PLL_R is the reference divider selected by C5[PRDIV0]
   *  M is the multiplier selected by C6[VDIV0]
   *
   * Then the core/system and bus/flash clocks are divided:
   *   f_SYS = f_MCGOUTCLK / OUTDIV1 = 96 MHz / 2 = 48 MHz
   *   f_BUS = f_MCGOUTCLK / OUTDIV1 / OUTDIV4 = 96 MHz / 4 = 24 MHz
   */

  /* The MCGOUTCLK is divided by OUTDIV1 and OUTDIV4:
   * OUTDIV1 (divider for core/system and bus/flash clock)
   * OUTDIV4 (additional divider for bus/flash clock) */
  SIM->CLKDIV1 =
          SIM_CLKDIV1_OUTDIV1(1) |  /* OUTDIV1 = divide-by-2 => 48 MHz */
          SIM_CLKDIV1_OUTDIV4(1);   /* OUTDIV4 = divide-by-2 => 24 MHz */

  SIM->SOPT2 =
          SIM_SOPT2_TPMSRC(1) | /* MCGFLLCLK clock or MCGPLLCLK/2 */
          SIM_SOPT2_PLLFLLSEL;  /* PLLFLLSEL=MCGPLLCLK/2 */

  /* EXTAL0 and XTAL0 */
  PORTA->PCR[18] &= ~0x01000700; /* Set PA18 to analog (default) */
  PORTA->PCR[19] &= ~0x01000700; /* Set PA19 to analog (default) */

  OSC0->CR = 0;

  /* From KL25P80M48SF0RM section 24.5.1.1 "Initializing the MCG". */
  /* To change from FEI mode to FBE mode: */
  /* (1) Select the external clock source in C2 register.
         Use low-power OSC mode (HGO0=0) which enables internal feedback
         resistor since FRDM-KL25Z has feedback resistor R25 unpopulated.
         Use high-gain mode by setting C2[HGO0] instead if external
         feedback resistor Rf is installed.  */
  MCG->C2 =
          MCG_C2_RANGE0(2) |  /* very high frequency range */
          0;                  /* external reference (no crystal) */
  /* (2) Write to C1 to select the clock mode. */
  MCG->C1 = /* Clear the IREFS bit to switch to the external reference. */
          MCG_C1_CLKS_ERCLK |  /* Use ERCLK for system clock, MCGCLKOUT. */
          MCG_C1_FRDIV(5);     /* Divide ERCLK / 1024 for FLL reference. */
  /* Note: FLL reference frequency must be 31.25 kHz to 39.0625 kHz.
     32 MHz / 1024 = 31.25 kHz. */
  MCG->C4 &= ~(MCG_C4_DMX32 | MCG_C4_DRST_DRS_MASK);
  MCG->C6 = 0;  /* PLLS=0: Select FLL as MCG source, not PLL */

  /* (3) Once configuration is set, wait for MCG mode change. */

  /* From KL25P80M48SF0RM section 24.5.31: */
  /* (1)(d) Loop until S[IREFST] is 0, indicating the
     external reference is the current reference clock source. */
  while ((MCG->S & MCG_S_IREFST) != 0)
    ;  /* Wait until external reference clock is FLL reference. */
  /* (1)(e) Loop until S[CLKST] is 2'b10, indicating
     the external reference clock is selected to feed MCGOUTCLK. */
  while ((MCG->S & MCG_S_CLKST_MASK) != MCG_S_CLKST_ERCLK)
    ;  /* Wait until external reference clock has been selected. */

  /* --- MCG mode: FBE (FLL bypassed, external crystal) ---
     Now the MCG is in FBE mode.
     Although the FLL is bypassed, it is still on. */

  /* (2)    Then configure C5[PRDIV0] to generate the
     correct PLL reference frequency. */
  MCG->C5 = MCG_C5_PRDIV0(7);  /* PLL External Reference Divide by 8 (4 MHz)*/
  /* (3)    Then from FBE transition to PBE mode. */
  /* (3)(b) C6[PLLS]=1 to select PLL. */
  /* (3)(b) C6[VDIV0]=5'b0000 (x24) 2 MHz * 24 = 48 MHz. */
  MCG->C6 = MCG_C6_PLLS | MCG_C6_VDIV0(0) | MCG_C6_CME0;
  /* (3)(d) Loop until S[PLLST], indicating PLL
     is the PLLS clock source. */
  while ((MCG->S & MCG_S_PLLST) == 0)
    ;  /* wait until PLL is the PLLS clock source. */
  /* (3)(e) Loop until S[LOCK0] is set, indicating the PLL has acquired lock. */
  /* PLL selected as MCG source. VDIV0=00000 (Multiply=24). */
  while ((MCG->S & MCG_S_LOCK0) == 0)
    ;  /* wait until PLL locked */

  /* --- MCG mode: PBE (PLL bypassed, external crystal) --- */

  /* (4)    Transition from PBE mode to PEE mode. */
  /* (4)(a) C1[CLKS] = 2'b00 to select PLL output as system clock source. */
  // Switch to PEE mode
  //    Select PLL output (CLKS=0)
  //    FLL external reference divider (FRDIV=5)
  //    External reference clock for FLL (IREFS=0)
  MCG->C1 = MCG_C1_CLKS(0) |
            MCG_C1_FRDIV(5);
  /* (4)(b) Loop until S[CLKST] are 2'b11, indicating the PLL output is selected for MCGOUTCLK. */
  while ((MCG->S & MCG_S_CLKST_MASK) != MCG_S_CLKST_PLL)
    ;  /* wait until clock switched to PLL output */

  /* --- MCG mode: PEE (PLL enabled, external crystal) --- */
}
