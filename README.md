# stm32_watch

watch_test_freertos is a cubeIDE project that shows all watch functionality
(besides bluetooth comms) working in the freertos environment; we have the
touch sensor reporting results, screen images, text, and clock values; button
interrupt based task notifications; vibration control; and the RTC clock
updating appropriately and used for screen output.  The touch sensor library
has been modified so it doesn't run in 'turbo mode' since we dont need to
sample at that speed.
