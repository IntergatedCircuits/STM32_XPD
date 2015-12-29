# STM32 eXtensible Peripheral Drivers
## A new approach for peripheral access
This project was created to provide a convenient and efficient C programming toolset for handling STM32 series microcontroller peripherals. The STM32 device peripheral decriptor header files are parsed and regenerated with a custom program to grant more efficient and readable peripheral access to the programmer. The core functionalities are detailed below:
### 1 Register structures with bitfields
The new peripheral structures come with their registers' bit field representations defined in addition to the regular register word access.
To illustrate the benefits of the introduced changes, a CMSIS standard compliant extract of the Analog Comparator (COMP) of the STM32 family is presented:
```C
/* extract of CMSIS standard compliant device peripheral header */
typedef struct
{
  __IO uint32_t CSR;      /*!< Comparator control Status register, Address offset: 0x00 */
} COMP_TypeDef;

#define COMP1             ((COMP_TypeDef *) COMP1_BASE)

#define COMP_CSR_COMPxEN  ((uint32_t)0x00000001) /*!< COMPx enable */

/* usage example */
/* enable the comparator */
void comp_enable(COMP_TypeDef * comp)
{
  comp->CSR |= COMP_CSR_COMPxEN;
}
/* disable the comparator */
void comp_disable(COMP_TypeDef * comp)
{
  comp->CSR &= ~COMP_CSR_COMPxEN;
}
```
It can be observed that the register bits are set and reset using bit masking values that are defined in the device header. This project provides a syntactically better solution for register bit manipulations:
```C
/* extract of peripheral header used in STM32_XPD */
typedef struct {
    union {
        struct {
            __IO uint32_t EN : 1;               /*!< COMPx enable */
            __IO uint32_t SW1 : 1;              /*!< COMPx SW1 switch control */
            __IO uint32_t MODE : 2;             /*!< COMPx power mode */
            __IO uint32_t INSEL : 3;            /*!< COMPx inverting input select */
            __IO uint32_t NONINSEL : 1;         /*!< COMPx non inverting input select */
                 uint32_t __RESERVED0 : 1;
            __IO uint32_t WNDWEN : 1;           /*!< COMPx window mode enable */
            __IO uint32_t OUTSEL : 4;           /*!< COMPx output select */
                 uint32_t __RESERVED1 : 1;
            __IO uint32_t POL : 1;              /*!< COMPx output polarity */
            __IO uint32_t HYST : 2;             /*!< COMPx hysteresis */
            __IO uint32_t BLANKING : 2;         /*!< COMPx blanking */
                 uint32_t __RESERVED2 : 10;
            __IO uint32_t OUT : 1;              /*!< COMPx output level */
            __IO uint32_t LOCK : 1;             /*!< COMPx lock */
        } b;
        __IO uint32_t w;
    } CSR;                                   /*!< Comparator control Status register, Address offset: 0x00 */
} COMP_TypeDef;

#define COMP1             ((COMP_TypeDef *) COMP1_BASE)

/* usage example */
/* enable the comparator */
void comp_enable(COMP_TypeDef * comp)
{
  comp->CSR.b.EN = 1;
}
/* disable the comparator */
void comp_disable(COMP_TypeDef * comp)
{
  comp->CSR.b.EN = 0;
}
```
This approach is much more programmer-friendly, both in terms of coding and decoding, and in fact it is the preferred way of dealing with microcontroller peripherals in C code.
### 2 Full bit-band support
The Cortex M3, M4 and M7 cores enable the first 1MB of the SRAM and peripheral address space to be addressed bitwise, using a bit remapping address space called the alias region. A detailed explanation on the subject is available [here](/BitBanding.md). The project provides structures that make it possible to execute atomic bit reading and writing operations on the peripherals that are in the specified address range. The following example shows how bit-banding can be utilized in the same context as above:
```C
/* extract of core header used in STM32_XPD */
#define PERIPH_BB(Address)		((void *)(0x42000000 | (((uint32_t)(Address) & 0x000FFFFF) << 5)))

/* extract of peripheral header used in STM32_XPD */
typedef struct {
    struct {
        __IO uint32_t EN;                   /*!< COMPx enable */
        __IO uint32_t SW1;                  /*!< COMPx SW1 switch control */
        __IO uint32_t MODE[2];              /*!< COMPx power mode */
        __IO uint32_t INSEL[3];             /*!< COMPx inverting input select */
        __IO uint32_t NONINSEL;             /*!< COMPx non inverting input select */
             uint32_t __RESERVED0;
        __IO uint32_t WNDWEN;               /*!< COMPx window mode enable */
        __IO uint32_t OUTSEL[4];            /*!< COMPx output select */
             uint32_t __RESERVED1;
        __IO uint32_t POL;                  /*!< COMPx output polarity */
        __IO uint32_t HYST[2];              /*!< COMPx hysteresis */
        __IO uint32_t BLANKING[2];          /*!< COMPx blanking */
             uint32_t __RESERVED2[10];
        __IO uint32_t OUT;                  /*!< COMPx output level */
        __IO uint32_t LOCK;                 /*!< COMPx lock */
    } CSR;                                  /*!< Comparator control Status register, Address offset: 0x00 */
} COMP_BitBand_TypeDef;

#define COMP1             ((COMP_TypeDef *) COMP1_BASE)

#define COMP_BB(inst)     ((COMP_BitBand_TypeDef *) PERIPH_BB(inst))

/* usage example */
COMP_BitBand_TypeDef * COMP1_bb = COMP_BB(COMP1);

/* enable the comparator */
void comp_enable(COMP_BitBand_TypeDef * comp_bb)
{
  comp_bb->CSR.EN = 1;
}
/* disable the comparator */
void comp_disable(COMP_BitBand_TypeDef * comp_bb)
{
  comp_bb->CSR.EN = 0;
}
```
