#ifndef PALAWAN_H_
#define PALAWAN_H_

enum palawan_model {
    palawan_unknown = 0,
    palawan_tx = 1,
    palawan_rx = 2,
};

enum palawan_model palawanModel(void);

#endif /* PALAWAN_H_ */
