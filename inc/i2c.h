#ifndef I2C_H_
#define I2C_H_

#include "stm32f413xx.h"

struct i2c {
  I2C_TypeDef *regs;
  IRQn_Type irqn;
  void (*enable_clock)(void);
  void (*init_gpio)(void);
  void (*reset_periphreal)(struct i2c *port);
};

#endif // I2C_H_