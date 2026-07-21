ENTRY(ESP32Reset)

PROVIDE(__pre_init = DefaultPreInit);
PROVIDE(__zero_bss = default_mem_hook);
PROVIDE(__init_data = default_mem_hook);
PROVIDE(__post_init = default_post_init);

PROVIDE(__level_1_interrupt = handle_interrupts);
PROVIDE(__level_2_interrupt = handle_interrupts);
PROVIDE(__level_3_interrupt = handle_interrupts);

INCLUDE exception.x

SECTIONS {
  .rwdata_dummy (NOLOAD) :
  {
    . = ALIGN(ALIGNOF(.rwtext));
    . = . + MAX(SIZEOF(.rwtext) + SIZEOF(.rwtext.wifi) + RESERVE_ICACHE + VECTORS_SIZE, 32k) - 32k;
    . = ALIGN(4);
    _rwdata_reserved_start = .;
  } > RWDATA
}
INSERT BEFORE .data;

INCLUDE "fixups/rodata_dummy.x"

INCLUDE "rwtext.x"
INCLUDE "text.x"
INCLUDE "rwdata.x"
INCLUDE "coffee-rodata.x"
INCLUDE "rtc_fast.x"
INCLUDE "rtc_slow.x"
INCLUDE "stack.x"
INCLUDE "dram2.x"

EXTERN(DefaultHandler);

INCLUDE "device.x"
