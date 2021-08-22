//
// Created by Admin on 14.06.2021.
//

#ifndef TTGO_T_BEAM_LORA_APRS_PSRAMJSONDOCUMENT_H
#define TTGO_T_BEAM_LORA_APRS_PSRAMJSONDOCUMENT_H

#include <ArduinoJson.h>

struct SpiRamAllocator {
  void* allocate(size_t size) {
    return heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
  }

  void deallocate(void* pointer) {
    heap_caps_free(pointer);
  }

  void* reallocate(void* ptr, size_t new_size) {
    return heap_caps_realloc(ptr, new_size, MALLOC_CAP_SPIRAM);
  }
};

using PSRAMJsonDocument = BasicJsonDocument<SpiRamAllocator>;


#endif //TTGO_T_BEAM_LORA_APRS_PSRAMJSONDOCUMENT_H
