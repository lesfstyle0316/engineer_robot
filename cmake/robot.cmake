# Application code lives only in bsp / user_driver / task.
# Dedicated HAL timebase supplied by BSP; SysTick is reserved for FreeRTOS.
get_target_property(ROBOT_MX_SOURCES ${CMAKE_PROJECT_NAME} SOURCES)
list(FILTER ROBOT_MX_SOURCES EXCLUDE REGEX "stm32f4xx_hal_timebase_tim[.]c$")
set_property(TARGET ${CMAKE_PROJECT_NAME} PROPERTY SOURCES ${ROBOT_MX_SOURCES})
target_compile_definitions(stm32cubemx INTERFACE HAL_TIM_MODULE_ENABLED)
file(GLOB ROBOT_BSP CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/bsp/src/*.c")
file(GLOB ROBOT_DRIVERS CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/user_driver/src/*.c")
set(ROBOT_TASKS ${CMAKE_SOURCE_DIR}/task/src/robot_core.c ${CMAKE_SOURCE_DIR}/task/src/robot_app.c)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${ROBOT_BSP} ${ROBOT_DRIVERS} ${ROBOT_TASKS})
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE bsp/inc user_driver/inc task/inc)
target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Wextra -Werror -Wno-unused-parameter)
target_link_libraries(${CMAKE_PROJECT_NAME} stm32cubemx m)
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>)
