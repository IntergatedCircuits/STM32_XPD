# Bit-banding in ARM Cortex cores

## What is bit-banding

A [bit-band operation](http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.ddi0337h/Behcjiic.html) lets a single load/store operation access a single bit in the memory. In the ARM Cortex M3, M4 and M7 cores, the bit-band operations are supported via separate address regions. The first 1MB of the SRAM and peripheral memory regions (**bit-band regions**) have separate 32MB **bit-band alias** address spaces. It means that in addition to being accessible as normal memory fields, their bits can be individually addressed and read/written.

Bit-banding does not mean that the selected regions have a special memory interface that would let them be accessed in a bitwise way. The CPU detects that a load/store operation uses the bit-band alias address, translates the address to the normal memory region and executes the operation as follows:
* Load: the memory word is read and the selected bit is shifted to the LSB.
* Store: the written bit is shifted to the selected position, then a READ-MODIFY-WRITE is performed on the memory word.


## Why it is useful

There are two main benefits to using bit-banding, **reduced number of operations** and **atomicity** of bit manipulation. 

* All masking operations normally associated with bit manipulation on word values can be excluded due to direct bit-accessibility. It takes only one operation (instead of 3) to modify a bit of a memory word, and two operations (read and compare, no masking AND) to determine if a bit field is set or not.
* As bit-band load/store operations are atomic, the bit fields of memory words can be modified by several concurrent threads without race condition, which can happen with normal READ-MODIFY-WRITE operations.



## How to use it
This is the part that in my opinion is not well thought through in the [official documentation](http://infocenter.arm.com/help/index.jsp?topic=/com.arm.doc.dai0179b/CHDJHIDF.html). It uses two macros to produce the separate SRAM and peripheral bit-band alias address based on the original address and the bit offset. This solution results in the recalculation of the alias address every time there is a load/store operation executed, therefore losing the speed advantage of bit-banding.

I have found a more efficient and elegant way of using bit-band alias addresses. **My solution is found [here](CMSIS/Include/core_cmBitband.h)**, it is proposed to be included in the CMSIS library. The transforming of the address to the alias region is separated for the SRAM and the peripheral addresses. They give the correct alias address for SRAM and peripheral bit-band memory regions. The programmer must ensure that the addresses used with these operations are in the bit-band region (typically the SRAM memory is smaller than its 1MB region, so the primary concern is not to use flash and out of range peripheral memory addresses). The macro *PERIPH_BB(Address)* is intended for use with peripheral addresses (which are fixed, known in compile time), while the inline function *SRAM_BB(Address)* is for SRAM addresses (which are determined at link time, therefore they should be calculated in run time).

In the alias region every bit has a word address (the least significant two address bits must always be zero). For this reason the alias addresses should only be used as word-size (32 bit) pointers, as this way the bit offset can be given as the index of the array represented by the alias pointer. The value read from the alias address can only be 1 or 0, and writing a value to an alias address will only consider the LSB of the value. Below are some typical use cases for separate peripheral and SRAM memory types.

### For SRAM addresses
As SRAM memory objects are only given address at link-time (unless it is fixed by linker flags and script), their alias addresses cannot be computed by the compiler, they must be calculated by the CPU during run time. The programmer should keep this difference in mind.
```C
/*
 * Calculates the Hamming-weight of a variable with a given bit range
 * (measured from the least significant bit) using the function parameter
 * on the SRAM stack.
 */
uint32_t BitCount(uint32_t Variable, uint8_t BitRange)
{
	// variable is on stack (in RAM), bit-band accessible
	uint32_t i, count = 0, *var_alias;
	var_alias = SRAM_BB(&Variable);

	if(BitRange > 32) BitRange = 32;

	for(i = 0; i < BitRange; i++)
		count += var_alias[i];
	return count;
}
```

### For peripheral addresses
In the case of peripheral addresses the alias address can be calculated by the compiler (using the macro), saving both program memory size and run time. A dummy GPIO peripheral is illustrated in the example, with some common bit manipulation uses. Observe the toggle function: the bit-band read operation can only return 0 or 1, and the bit-band write operation only stores the least significant bit.

```C
/* pseudo GPIO structure, every bit position corresponds to a GPIO pin */
typedef struct {
    __IO uint32 IDR; /* input data register */
    __IO uint32 ODR; /* output data register */
} GPIO_TypeDef;

typedef struct {
    __IO uint32 IDR[32]; /* input data register */
    __IO uint32 ODR[32]; /* output data register */
} GPIO_BitBand_TypeDef;

#define GPIO_BB(inst)             ((GPIO_BitBand_TypeDef *) PERIPH_BB(inst))

/* read bit at given register bit location */
inline uint32_t gpio_read_pin(GPIO_TypeDef * gpio, uint8_t pin)
{
	return GPIO_BB(gpio)->IDR[pin];
}
/* write bit at given register bit location */
inline void gpio_write_pin(GPIO_TypeDef * gpio, uint8_t pin, uint32_t value)
{
	GPIO_BB(gpio)->ODR[pin] = value;
}
/* toggle (invert) bit at given register bit location */
inline void gpio_toggle_pin(GPIO_TypeDef * gpio, uint8_t pin)
{
	GPIO_BB(gpio)->ODR[pin]++;
}
```
