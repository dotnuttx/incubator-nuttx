/****************************************************************************
 * boards/risc-v/k210/maix-bit/src/k210_gpio.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <syslog.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/irq.h>
#include <arch/irq.h>

#include <nuttx/ioexpander/gpio.h>

#include <arch/board/board.h>

#include "k210_fpioa.h"
#include "k210_gpiohs.h"

#if defined(CONFIG_DEV_GPIO) && !defined(CONFIG_GPIO_LOWER_HALF)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Pin 1 and 2 are used for this example as GPIO outputs. */

#define GPIO_OUT1  12
#define GPIO_OUT2  13

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct k210gpio_dev_s
{
  struct gpio_dev_s gpio;
  uint8_t id;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

#if BOARD_NGPIOOUT > 0
static int gpout_read(FAR struct gpio_dev_s *dev, FAR bool *value);
static int gpout_write(FAR struct gpio_dev_s *dev, bool value);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if BOARD_NGPIOOUT > 0
static const struct gpio_operations_s gpout_ops =
{
  .go_read   = gpout_read,
  .go_write  = gpout_write,
  .go_attach = NULL,
  .go_enable = NULL,
};

/* This array maps the GPIO pins used as OUTPUT */

static const uint32_t g_gpiooutputs[BOARD_NGPIOOUT] =
{
  GPIO_OUT1, GPIO_OUT2
};

static struct k210gpio_dev_s g_gpout[BOARD_NGPIOOUT];
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: gpout_read
 ****************************************************************************/

#if BOARD_NGPIOOUT > 0
static int gpout_read(FAR struct gpio_dev_s *dev, FAR bool *value)
{
  FAR struct k210gpio_dev_s *k210gpio =
    (FAR struct k210gpio_dev_s *)dev;

  DEBUGASSERT(k210gpio != NULL && value != NULL);
  DEBUGASSERT(k210gpio->id < BOARD_NGPIOOUT);
  gpioinfo("Reading...\n");

  *value = (int) k210_gpiohs_get_value(k210gpio->id + 1);
  return OK;
}

/****************************************************************************
 * Name: gpout_write
 ****************************************************************************/

static int gpout_write(FAR struct gpio_dev_s *dev, bool value)
{
  FAR struct k210gpio_dev_s *k210gpio =
    (FAR struct k210gpio_dev_s *)dev;

  DEBUGASSERT(k210gpio != NULL);
  DEBUGASSERT(k210gpio->id < BOARD_NGPIOOUT);
  gpioinfo("Writing %d\n", (int)value);

  k210_gpiohs_set_value(k210gpio->id + 1, !value);

  return OK;
}
#endif

#ifdef CONFIG_GPIO_LIB
// TODO: banks
static struct k210gpio_dev_s gpio_generic_bank0;

static int gp_lib_read(FAR struct gpio_dev_s *dev, FAR uint8_t pin,
                       FAR bool *value)
{
  FAR struct k210gpio_dev_s *k210gpio = (FAR struct k210gpio_dev_s *)dev;

  DEBUGASSERT(k210gpio != NULL && value != NULL);
  gpioinfo("Reading %d...\n", (int)pin);

  *value = (int) k210_gpiohs_get_value(pin);
  return OK;
}

static int gp_lib_write(FAR struct gpio_dev_s *dev, FAR uint8_t pin,
                        bool value)
{
  FAR struct k210gpio_dev_s *k210gpio = (FAR struct k210gpio_dev_s *)dev;

  DEBUGASSERT(k210gpio != NULL);
  gpioinfo("Writing %d to %d\n", (int)value, (int)pin);

  k210_gpiohs_set_value(pin, !value);
  return OK;
}

static int gp_lib_setpindir(FAR struct gpio_dev_s *dev, FAR uint8_t pin,
                            FAR enum gpio_pintype_e pintype)
{
  FAR struct k210gpio_dev_s *k210gpio = (FAR struct k210gpio_dev_s *)dev;
  uint32_t flags = K210_IOFLAG_GPIOHS;

  DEBUGASSERT(k210gpio != NULL);
  gpioinfo("Setting dir %d to %d\n", (int)pintype, (int)pin);

  if (pintype == GPIO_OUTPUT_PIN)
  {
    k210_fpioa_config(pin, (K210_IO_FUNC_GPIOHS0 + pin) | K210_IOFLAG_GPIOHS);
    k210_gpiohs_set_direction(pin, true);
  }
  else if (pintype == GPIO_INPUT_PIN_PULLDOWN)
  {
    flags = flags | K210_IO_PULL_DOWN;
    k210_fpioa_config(pin, (K210_IO_FUNC_GPIOHS0 + pin) | flags);
    k210_gpiohs_set_direction(pin, false);
  } else {
    return -EINVAL;
  }

  return OK;
}

static const struct gpio_lib_operations_s gpio_lib_ops =
{
  .gp_read      = gp_lib_read,
  .gp_write     = gp_lib_write,
  .gp_setpindir = gp_lib_setpindir,
};

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k210_gpio_init
 ****************************************************************************/

int k210_gpio_init(void)
{
  int i;
  int pincount = 0;

#if BOARD_NGPIOOUT > 0
  for (i = 0; i < BOARD_NGPIOOUT; i++)
    {
      /* Setup and register the GPIO pin */

      g_gpout[i].gpio.gp_pintype = GPIO_OUTPUT_PIN;
      g_gpout[i].gpio.gp_ops     = &gpout_ops;
      g_gpout[i].id              = i;
      gpio_pin_register(&g_gpout[i].gpio, pincount);

      /* Configure the pins that will be used as output */

      k210_fpioa_config(g_gpiooutputs[i],
                        (K210_IO_FUNC_GPIOHS1 + i) | K210_IOFLAG_GPIOHS);
      k210_gpiohs_set_direction(i + 1, true);
      k210_gpiohs_set_value(i + 1, true);

      pincount++;
    }
#endif

#ifdef CONFIG_GPIO_LIB
  gpio_generic_bank0.gpio.gp_pintype = GPIO_LIB_PIN;
  gpio_generic_bank0.gpio.gp_lib_ops = &gpio_lib_ops;
  gpio_generic_bank0.id = 0;

  gpio_pin_register(&gpio_generic_bank0.gpio, 0);
#endif

  return OK;
}
#endif /* CONFIG_DEV_GPIO && !CONFIG_GPIO_LOWER_HALF */
