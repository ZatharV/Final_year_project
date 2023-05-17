# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/Espressif/frameworks/esp-idf-v5.0.1/components/bootloader/subproject"
  "D:/nipun prev laptop/final_year/Project scripts/node_firmware/build/bootloader"
  "D:/nipun prev laptop/final_year/Project scripts/node_firmware/build/bootloader-prefix"
  "D:/nipun prev laptop/final_year/Project scripts/node_firmware/build/bootloader-prefix/tmp"
  "D:/nipun prev laptop/final_year/Project scripts/node_firmware/build/bootloader-prefix/src/bootloader-stamp"
  "D:/nipun prev laptop/final_year/Project scripts/node_firmware/build/bootloader-prefix/src"
  "D:/nipun prev laptop/final_year/Project scripts/node_firmware/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/nipun prev laptop/final_year/Project scripts/node_firmware/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/nipun prev laptop/final_year/Project scripts/node_firmware/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
