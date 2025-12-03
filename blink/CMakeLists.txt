    cmake_minimum_required(VERSION 3.13)

    # Pull in SDK (must be before project)
    include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

    # Pull in FreeRTOS
    include($ENV{FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

    project(app C CXX ASM)
    set(CMAKE_C_STANDARD 11)
    set(CMAKE_CXX_STANDARD 17)

    # Initialize the SDK
    pico_sdk_init()

    set(SRC_FLS
        main.c
        src/adc.c
        src/bcc.c
        src/fifo.c
        src/print.c
        src/rtc.c
        src/spiflash.c
        src/uart.c
        src/mutex.c
        src/globals.c
    )

    add_executable(blink ${SRC_FLS})

    target_include_directories(blink PRIVATE ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_CURRENT_LIST_DIR}/header)
    pico_set_binary_type(blink copy_to_ram)
    set_target_properties(blink PROPERTIES COMPILE_FLAGS "-Wall -Wextra")


    # pull in common dependencies
    target_link_libraries(blink pico_stdlib
                        pico_stdlib_headers
                        pico_multicore
                        hardware_structs
                        hardware_watchdog
                        hardware_uart
                        hardware_rtc
                        hardware_i2c
                        hardware_adc
                        hardware_dma
                        hardware_spi
                        hardware_flash
                        hardware_irq
                        hardware_timer
                        hardware_gpio
                        hardware_sync
                        FreeRTOS-Kernel
                        FreeRTOS-Kernel-Heap4
                        )

    # Enable/disable STDIO via USB and UART
    # We use UART1 as a debug port because UART0 is attached to RS485
    pico_enable_stdio_usb(blink 1)
    pico_enable_stdio_uart(blink 0)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(blink)
