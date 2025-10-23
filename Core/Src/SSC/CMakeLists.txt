# SSC_gen need implement to become actual lib
add_library(SSC)
target_sources(
  SSC
  PUBLIC ${CMAKE_CURRENT_LIST_DIR}/src/coeappl.c
         ${CMAKE_CURRENT_LIST_DIR}/src/ecatappl.c
         ${CMAKE_CURRENT_LIST_DIR}/src/ecatcoe.c
         ${CMAKE_CURRENT_LIST_DIR}/src/ecatslv.c
         # ${CMAKE_CURRENT_LIST_DIR}/src/el9800hw.c
         ${CMAKE_CURRENT_LIST_DIR}/src/mailbox.c
         ${CMAKE_CURRENT_LIST_DIR}/src/objdef.c
         ${CMAKE_CURRENT_LIST_DIR}/src/sdoserv.c
         ${CMAKE_CURRENT_LIST_DIR}/src/SSC-Device.c
         ssc_hal_stm_hal_impl.c
         ssc_appl_singleton.c)
target_compile_options(
  SSC PRIVATE -Wno-overflow -Wno-unused-variable -Wno-uninitialized
              -Wno-array-bounds -Wno-maybe-uninitialized -Wno-address)

target_include_directories(SSC PUBLIC ${CMAKE_CURRENT_LIST_DIR}/Src
                                      ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(SSC PRIVATE stm32cubemx)

# `siitool SSC-Device.xml -o SSC-Device.bin`

# `sudo ethercat sii_write SSC-Device.bin`
