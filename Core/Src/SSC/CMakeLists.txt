# SSC_gen need implement to become actual lib
add_library(SSC)
target_sources(
  SSC
  PUBLIC ${CMAKE_CURRENT_LIST_DIR}/Src/coeappl.c
         ${CMAKE_CURRENT_LIST_DIR}/Src/ecatappl.c
         ${CMAKE_CURRENT_LIST_DIR}/Src/ecatcoe.c
         ${CMAKE_CURRENT_LIST_DIR}/Src/ecatslv.c
         # ${CMAKE_CURRENT_LIST_DIR}/Src/el9800hw.c
         ${CMAKE_CURRENT_LIST_DIR}/Src/mailbox.c
         ${CMAKE_CURRENT_LIST_DIR}/Src/objdef.c
         ${CMAKE_CURRENT_LIST_DIR}/Src/sdoserv.c
         ${CMAKE_CURRENT_LIST_DIR}/Src/SSC-Device.c
         ssc_hal_stm_hal_impl.c
         ssc_appl_singleton.c)
target_compile_options(
  SSC PRIVATE -Wno-overflow -Wno-unused-variable -Wno-uninitialized
              -Wno-array-bounds -Wno-maybe-uninitialized -Wno-address)

target_include_directories(SSC PUBLIC ${CMAKE_CURRENT_LIST_DIR}/Src
                                      ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(SSC PRIVATE stm32cubemx)

# For ax58100, plz remember modify ESI eeprom tag word offset 0x02 `0x0A00` -->
# `0x1027` e.g. <ConfigData>050D08EE0A00</ConfigData> to <ConfigData>050D08EE1027</ConfigData>

# `siitool SSC-Device.xml -o SSC-Device.bin`

# `sudo ethercat sii_write SSC-Device.bin`
