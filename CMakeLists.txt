set(BOARD nrf52840_pca10056)

set(ENV{ZEPHYR_BASE} "/home/niranjan/workspace/assistr/zephyr-rots/zephyr")
set(ENV{ZEPHYR_SDK_INSTALL_DIR} "/home/niranjan/workspace/assistr/zephyr-rots/zephyr-sdk")
set(ENV{ZEPHYR_TOOLCHAIN_VARIANT} "zephyr" )

include ($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(NONE)
target_sources(app PRIVATE src/main.c)