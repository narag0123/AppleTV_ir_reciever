# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/jude/LocalStorage/SDK/esp/esp-idf/components/bootloader/subproject"
  "/Users/jude/LocalStorage/Embedded Project/Acton Activator/firmware/ir_reciever/cmake-build-debug/bootloader"
  "/Users/jude/LocalStorage/Embedded Project/Acton Activator/firmware/ir_reciever/cmake-build-debug/bootloader-prefix"
  "/Users/jude/LocalStorage/Embedded Project/Acton Activator/firmware/ir_reciever/cmake-build-debug/bootloader-prefix/tmp"
  "/Users/jude/LocalStorage/Embedded Project/Acton Activator/firmware/ir_reciever/cmake-build-debug/bootloader-prefix/src/bootloader-stamp"
  "/Users/jude/LocalStorage/Embedded Project/Acton Activator/firmware/ir_reciever/cmake-build-debug/bootloader-prefix/src"
  "/Users/jude/LocalStorage/Embedded Project/Acton Activator/firmware/ir_reciever/cmake-build-debug/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/jude/LocalStorage/Embedded Project/Acton Activator/firmware/ir_reciever/cmake-build-debug/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/jude/LocalStorage/Embedded Project/Acton Activator/firmware/ir_reciever/cmake-build-debug/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
