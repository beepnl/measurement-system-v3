# 1 "D:\\localrep\\Beep\\BeeMonitor\\Code\\nRF\\nRF5_SDK_15.3\\modules\\nrfx\\mdk\\ses_startup_nrf52840.s"
# 1 "<built-in>"
# 1 "<command-line>"
# 1 "D:\\localrep\\Beep\\BeeMonitor\\Code\\nRF\\nRF5_SDK_15.3\\modules\\nrfx\\mdk\\ses_startup_nrf52840.s"
# 53 "D:\\localrep\\Beep\\BeeMonitor\\Code\\nRF\\nRF5_SDK_15.3\\modules\\nrfx\\mdk\\ses_startup_nrf52840.s"
  .syntax unified
  .code 16

  .section .init, "ax"
  .align 0






  .thumb_func
  .weak NMI_Handler
NMI_Handler:
  b .

  .thumb_func
  .weak HardFault_Handler
HardFault_Handler:
  b .

  .thumb_func
  .weak MemoryManagement_Handler
MemoryManagement_Handler:
  b .

  .thumb_func
  .weak BusFault_Handler
BusFault_Handler:
  b .

  .thumb_func
  .weak UsageFault_Handler
UsageFault_Handler:
  b .

  .thumb_func
  .weak SVC_Handler
SVC_Handler:
  b .

  .thumb_func
  .weak DebugMon_Handler
DebugMon_Handler:
  b .

  .thumb_func
  .weak PendSV_Handler
PendSV_Handler:
  b .

  .thumb_func
  .weak SysTick_Handler
SysTick_Handler:
  b .

  .thumb_func
  .weak Dummy_Handler
Dummy_Handler:
  b .





.weak POWER_CLOCK_IRQHandler
.thumb_set POWER_CLOCK_IRQHandler, Dummy_Handler

.weak RADIO_IRQHandler
.thumb_set RADIO_IRQHandler, Dummy_Handler

.weak UARTE0_UART0_IRQHandler
.thumb_set UARTE0_UART0_IRQHandler, Dummy_Handler

.weak SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler
.thumb_set SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler, Dummy_Handler

.weak SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler
.thumb_set SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler, Dummy_Handler

.weak NFCT_IRQHandler
.thumb_set NFCT_IRQHandler, Dummy_Handler

.weak GPIOTE_IRQHandler
.thumb_set GPIOTE_IRQHandler, Dummy_Handler

.weak SAADC_IRQHandler
.thumb_set SAADC_IRQHandler, Dummy_Handler

.weak TIMER0_IRQHandler
.thumb_set TIMER0_IRQHandler, Dummy_Handler

.weak TIMER1_IRQHandler
.thumb_set TIMER1_IRQHandler, Dummy_Handler

.weak TIMER2_IRQHandler
.thumb_set TIMER2_IRQHandler, Dummy_Handler

.weak RTC0_IRQHandler
.thumb_set RTC0_IRQHandler, Dummy_Handler

.weak TEMP_IRQHandler
.thumb_set TEMP_IRQHandler, Dummy_Handler

.weak RNG_IRQHandler
.thumb_set RNG_IRQHandler, Dummy_Handler

.weak ECB_IRQHandler
.thumb_set ECB_IRQHandler, Dummy_Handler

.weak CCM_AAR_IRQHandler
.thumb_set CCM_AAR_IRQHandler, Dummy_Handler

.weak WDT_IRQHandler
.thumb_set WDT_IRQHandler, Dummy_Handler

.weak RTC1_IRQHandler
.thumb_set RTC1_IRQHandler, Dummy_Handler

.weak QDEC_IRQHandler
.thumb_set QDEC_IRQHandler, Dummy_Handler

.weak COMP_LPCOMP_IRQHandler
.thumb_set COMP_LPCOMP_IRQHandler, Dummy_Handler

.weak SWI0_EGU0_IRQHandler
.thumb_set SWI0_EGU0_IRQHandler, Dummy_Handler

.weak SWI1_EGU1_IRQHandler
.thumb_set SWI1_EGU1_IRQHandler, Dummy_Handler

.weak SWI2_EGU2_IRQHandler
.thumb_set SWI2_EGU2_IRQHandler, Dummy_Handler

.weak SWI3_EGU3_IRQHandler
.thumb_set SWI3_EGU3_IRQHandler, Dummy_Handler

.weak SWI4_EGU4_IRQHandler
.thumb_set SWI4_EGU4_IRQHandler, Dummy_Handler

.weak SWI5_EGU5_IRQHandler
.thumb_set SWI5_EGU5_IRQHandler, Dummy_Handler

.weak TIMER3_IRQHandler
.thumb_set TIMER3_IRQHandler, Dummy_Handler

.weak TIMER4_IRQHandler
.thumb_set TIMER4_IRQHandler, Dummy_Handler

.weak PWM0_IRQHandler
.thumb_set PWM0_IRQHandler, Dummy_Handler

.weak PDM_IRQHandler
.thumb_set PDM_IRQHandler, Dummy_Handler

.weak MWU_IRQHandler
.thumb_set MWU_IRQHandler, Dummy_Handler

.weak PWM1_IRQHandler
.thumb_set PWM1_IRQHandler, Dummy_Handler

.weak PWM2_IRQHandler
.thumb_set PWM2_IRQHandler, Dummy_Handler

.weak SPIM2_SPIS2_SPI2_IRQHandler
.thumb_set SPIM2_SPIS2_SPI2_IRQHandler, Dummy_Handler

.weak RTC2_IRQHandler
.thumb_set RTC2_IRQHandler, Dummy_Handler

.weak I2S_IRQHandler
.thumb_set I2S_IRQHandler, Dummy_Handler

.weak FPU_IRQHandler
.thumb_set FPU_IRQHandler, Dummy_Handler

.weak USBD_IRQHandler
.thumb_set USBD_IRQHandler, Dummy_Handler

.weak UARTE1_IRQHandler
.thumb_set UARTE1_IRQHandler, Dummy_Handler

.weak QSPI_IRQHandler
.thumb_set QSPI_IRQHandler, Dummy_Handler

.weak CRYPTOCELL_IRQHandler
.thumb_set CRYPTOCELL_IRQHandler, Dummy_Handler

.weak PWM3_IRQHandler
.thumb_set PWM3_IRQHandler, Dummy_Handler

.weak SPIM3_IRQHandler
.thumb_set SPIM3_IRQHandler, Dummy_Handler





  .extern Reset_Handler
  .global nRFInitialize

  .thumb_func
nRFInitialize:
  bx lr






  .section .vectors, "ax"
  .align 0
  .global _vectors
  .extern __stack_end__

_vectors:
  .word __stack_end__
  .word Reset_Handler
  .word NMI_Handler
  .word HardFault_Handler
  .word MemoryManagement_Handler
  .word BusFault_Handler
  .word UsageFault_Handler
  .word 0
  .word 0
  .word 0
  .word 0
  .word SVC_Handler
  .word DebugMon_Handler
  .word 0
  .word PendSV_Handler
  .word SysTick_Handler


  .word POWER_CLOCK_IRQHandler
  .word RADIO_IRQHandler
  .word UARTE0_UART0_IRQHandler
  .word SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQHandler
  .word SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQHandler
  .word NFCT_IRQHandler
  .word GPIOTE_IRQHandler
  .word SAADC_IRQHandler
  .word TIMER0_IRQHandler
  .word TIMER1_IRQHandler
  .word TIMER2_IRQHandler
  .word RTC0_IRQHandler
  .word TEMP_IRQHandler
  .word RNG_IRQHandler
  .word ECB_IRQHandler
  .word CCM_AAR_IRQHandler
  .word WDT_IRQHandler
  .word RTC1_IRQHandler
  .word QDEC_IRQHandler
  .word COMP_LPCOMP_IRQHandler
  .word SWI0_EGU0_IRQHandler
  .word SWI1_EGU1_IRQHandler
  .word SWI2_EGU2_IRQHandler
  .word SWI3_EGU3_IRQHandler
  .word SWI4_EGU4_IRQHandler
  .word SWI5_EGU5_IRQHandler
  .word TIMER3_IRQHandler
  .word TIMER4_IRQHandler
  .word PWM0_IRQHandler
  .word PDM_IRQHandler
  .word 0
  .word 0
  .word MWU_IRQHandler
  .word PWM1_IRQHandler
  .word PWM2_IRQHandler
  .word SPIM2_SPIS2_SPI2_IRQHandler
  .word RTC2_IRQHandler
  .word I2S_IRQHandler
  .word FPU_IRQHandler
  .word USBD_IRQHandler
  .word UARTE1_IRQHandler
  .word QSPI_IRQHandler
  .word CRYPTOCELL_IRQHandler
  .word 0
  .word 0
  .word PWM3_IRQHandler
  .word 0
  .word SPIM3_IRQHandler
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
  .word 0
_vectors_end:
