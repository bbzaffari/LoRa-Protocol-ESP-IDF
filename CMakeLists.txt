idf_component_register(
    SRCS lora_proto.c
    INCLUDE_DIRS .
    REQUIRES lora driver freertos log esp_timer
)
