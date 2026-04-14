cmake_minimum_required(VERSION 3.16)

idf_component_register(
    SRCS
        "src/adin1110.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        driver
        esp_netif
        freertos
        log
        heap
)
