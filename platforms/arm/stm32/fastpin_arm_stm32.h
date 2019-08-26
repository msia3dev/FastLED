#ifndef __FASTPIN_ARM_STM32_H
#define __FASTPIN_ARM_STM32_H

FASTLED_NAMESPACE_BEGIN

#if defined(FASTLED_FORCE_SOFTWARE_PINS)
#warning "Software pin support forced, pin access will be sloightly slower."
#define NO_HARDWARE_PIN_SUPPORT
#undef HAS_HARDWARE_PIN_SUPPORT

#else

/// Template definition for STM32 style ARM pins, providing direct access to the various GPIO registers.  Note that this
/// uses the full port GPIO registers.  In theory, in some way, bit-band register access -should- be faster, however I have found
/// that something about the way gcc does register allocation results in the bit-band code being slower.  It will need more fine tuning.
/// The registers are data output, set output, clear output, toggle output, input, and direction

template<uint8_t PIN, uint8_t _BIT, uint32_t _MASK, typename _GPIO> class _ARMPIN {
public:
  typedef volatile uint32_t * port_ptr_t;
  typedef uint32_t port_t;

  #if 0
  inline static void setOutput() {
    if(_BIT<8) {
      _CRL::r() = (_CRL::r() & (0xF << (_BIT*4)) | (0x1 << (_BIT*4));
    } else {
      _CRH::r() = (_CRH::r() & (0xF << ((_BIT-8)*4))) | (0x1 << ((_BIT-8)*4));
    }
  }
  inline static void setInput() { /* TODO */ } // TODO: preform MUX config { _PDDR::r() &= ~_MASK; }
  #endif

  inline static void setOutput() { pinMode(PIN, OUTPUT); } // TODO: perform MUX config { _PDDR::r() |= _MASK; }
  inline static void setInput() { pinMode(PIN, INPUT); } // TODO: preform MUX config { _PDDR::r() &= ~_MASK; }

  inline static void hi() __attribute__ ((always_inline)) { _GPIO::r()->BSRR = _MASK; }
  inline static void lo() __attribute__ ((always_inline)) { _GPIO::r()->BRR = _MASK; }
  // inline static void lo() __attribute__ ((always_inline)) { _GPIO::r()->BSRR = (_MASK<<16); }
  inline static void set(register port_t val) __attribute__ ((always_inline)) { _GPIO::r()->ODR = val; }

  inline static void strobe() __attribute__ ((always_inline)) { toggle(); toggle(); }

  inline static void toggle() __attribute__ ((always_inline)) { if(_GPIO::r()->ODR & _MASK) { lo(); } else { hi(); } }

  inline static void hi(register port_ptr_t port) __attribute__ ((always_inline)) { hi(); }
  inline static void lo(register port_ptr_t port) __attribute__ ((always_inline)) { lo(); }
  inline static void fastset(register port_ptr_t port, register port_t val) __attribute__ ((always_inline)) { *port = val; }

  inline static port_t hival() __attribute__ ((always_inline)) { return _GPIO::r()->ODR | _MASK; }
  inline static port_t loval() __attribute__ ((always_inline)) { return _GPIO::r()->ODR & ~_MASK; }
  inline static port_ptr_t port() __attribute__ ((always_inline)) { return &_GPIO::r()->ODR; }
  inline static port_ptr_t sport() __attribute__ ((always_inline)) { return &_GPIO::r()->BSRR; }
  inline static port_ptr_t cport() __attribute__ ((always_inline)) { return &_GPIO::r()->BRR; }
  inline static port_t mask() __attribute__ ((always_inline)) { return _MASK; }
};

#if defined(STM32F10X_MD)
  #define _R(T) struct __gen_struct_ ## T
  #define _RD32(T) struct __gen_struct_ ## T { static __attribute__((always_inline)) inline volatile GPIO_TypeDef * r() { return T; } };
  #define _FL_IO(L,C) _RD32(GPIO ## L);  _FL_DEFINE_PORT3(L, C, _R(GPIO ## L));
#elif defined(__STM32F1__)
  #define _R(T) struct __gen_struct_ ## T
  #define _DEFPIN_ARM(PIN, BIT, L) template<> class FastPin<PIN> : public _ARMPIN<PIN, BIT, 1 << BIT, _R(GPIO ## L)> {};
  #define _RD32(T) struct __gen_struct_ ## T { static __attribute__((always_inline)) inline gpio_reg_map* r() { return T->regs; } };
  #define _IO32(L) _RD32(GPIO ## L)
#else
 #error "Platform not supported"
#endif

#define _FL_DEFPIN(PIN, BIT, L) template<> class FastPin<PIN> : public _ARMPIN<PIN, BIT, 1 << BIT, _R(GPIO ## L)> {};

#ifdef GPIOA
_FL_IO(A,0);
#endif
#ifdef GPIOB
_FL_IO(B,1);
#endif
#ifdef GPIOC
_FL_IO(C,2);
#endif
#ifdef GPIOD
_FL_IO(D,3);
#endif
#ifdef GPIOE
_FL_IO(E,4);
#endif
#ifdef GPIOF
_FL_IO(F,5);
#endif
#ifdef GPIOG
_FL_IO(G,6);
#endif

// Actual pin definitions
#if defined(SPARK) // Sparkfun STM32F103 based board



#define MAX_PIN 19
_FL_DEFPIN(0, 7, B);
_FL_DEFPIN(1, 6, B);
_FL_DEFPIN(2, 5, B);
_FL_DEFPIN(3, 4, B);
_FL_DEFPIN(4, 3, B);
_FL_DEFPIN(5, 15, A);
_FL_DEFPIN(6, 14, A);
_FL_DEFPIN(7, 13, A);
_FL_DEFPIN(8, 8, A);
_FL_DEFPIN(9, 9, A);
_FL_DEFPIN(10, 0, A);
_FL_DEFPIN(11, 1, A);
_FL_DEFPIN(12, 4, A);
_FL_DEFPIN(13, 5, A);
_FL_DEFPIN(14, 6, A);
_FL_DEFPIN(15, 7, A);
_FL_DEFPIN(16, 0, B);
_FL_DEFPIN(17, 1, B);
_FL_DEFPIN(18, 3, A);
_FL_DEFPIN(19, 2, A);


#define SPI_DATA 15
#define SPI_CLOCK 13

#define HAS_HARDWARE_PIN_SUPPORT

#endif // SPARK

#if defined(__STM32F1__) // Generic STM32F103 aka "Blue Pill"

_IO32(A); _IO32(B); _IO32(C); _IO32(D);

#define MAX_PIN PB1

_DEFPIN_ARM(PB11, 11, B);
_DEFPIN_ARM(PB10, 10, B);
_DEFPIN_ARM(PB2, 2, B);
_DEFPIN_ARM(PB0, 0, B);
_DEFPIN_ARM(PA7, 7, A);
_DEFPIN_ARM(PA6, 6, A);
_DEFPIN_ARM(PA5, 5, A);
_DEFPIN_ARM(PA4, 4, A);
_DEFPIN_ARM(PA3, 3, A);
_DEFPIN_ARM(PA2, 2, A);
_DEFPIN_ARM(PA1, 1, A);
_DEFPIN_ARM(PA0, 0, A);
_DEFPIN_ARM(PC15, 15, C);
_DEFPIN_ARM(PC14, 14, C);
_DEFPIN_ARM(PC13, 13, C);
_DEFPIN_ARM(PB7, 7, B);
_DEFPIN_ARM(PB6, 6, B);
_DEFPIN_ARM(PB5, 5, B);
_DEFPIN_ARM(PB4, 4, B);
_DEFPIN_ARM(PB3, 3, B);
_DEFPIN_ARM(PA15, 15, A);
_DEFPIN_ARM(PA14, 14, A);
_DEFPIN_ARM(PA13, 13, A);
_DEFPIN_ARM(PA12, 12, A);
_DEFPIN_ARM(PA11, 11, A);
_DEFPIN_ARM(PA10, 10, A);
_DEFPIN_ARM(PA9, 9, A);
_DEFPIN_ARM(PA8, 8, A);
_DEFPIN_ARM(PB15, 15, B);
_DEFPIN_ARM(PB14, 14, B);
_DEFPIN_ARM(PB13, 13, B);
_DEFPIN_ARM(PB12, 12, B);
_DEFPIN_ARM(PB8, 8, B);
_DEFPIN_ARM(PB9, 9, B);
_DEFPIN_ARM(PB1, 1, B);

#define SPI_DATA BOARD_SPI1_MOSI_PIN
#define SPI_CLOCK BOARD_SPI1_SCK_PIN

#define HAS_HARDWARE_PIN_SUPPORT

#endif // __STM32F1__

#endif // FASTLED_FORCE_SOFTWARE_PINS

FASTLED_NAMESPACE_END

#endif // __INC_FASTPIN_ARM_STM32
