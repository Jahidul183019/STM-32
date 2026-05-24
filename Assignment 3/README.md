# Assignment 3

This folder contains the bare-metal BMP280 sensor projects for Assignment 3 on the NUCLEO-F446RE.

## Board Pinout Reference

![NUCLEO-F446RE pinout reference](assets/nucleo-f446re-pinout.svg)

The reference image is stored in the repo at `Assignment 3/assets/nucleo-f446re-pinout.svg`.

## Part A - BMP280 over SPI

Part A uses the BMP280 in SPI mode.

| Signal | STM32F446RE pin | Mode | Notes |
| --- | --- | --- | --- |
| SPI2_MOSI | PC1 | AF7 | Data out to BMP280 |
| SPI2_MISO | PC2 | AF5 | Data in from BMP280 |
| SPI2_SCK | PC7 | AF5 | SPI clock |
| CS | PB9 | GPIO output | Active low chip select |
| USART2_TX | PA2 | AF7 | Serial log output |
| USART2_RX | PA3 | AF7 | Serial log input |

## Part B - BMP280 over I2C

Part B uses the BMP280 in I2C mode.

| Signal | STM32F446RE pin | Mode | Notes |
| --- | --- | --- | --- |
| I2C1_SCL | PB6 | AF4 | Open-drain with pull-up |
| I2C1_SDA | PB7 | AF4 | Open-drain with pull-up |
| USART2_TX | PA2 | AF7 | Serial log output |
| USART2_RX | PA3 | AF7 | Serial log input |

## Project Map

- [Part A BareMetal](PartA_BareMetal)
- [Part B BareMetal](PartB_BareMetal)
