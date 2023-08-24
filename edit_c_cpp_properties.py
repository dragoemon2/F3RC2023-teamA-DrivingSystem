import json
import time

#実際のincludePath
include_path = [
    "${workspaceRoot}/include",
    "${workspaceRoot}/src",
    "~/.platformio/packages/framework-mbed/",
    "${workspaceRoot}/.pio/build/nucleo_f446re",
    "~/.platformio/packages/framework-mbed/platform/include/",
    "~/.platformio/packages/framework-mbed/cmsis/",
    "~/.platformio/packages/framework-mbed/connectivity/",
    "~/.platformio/packages/framework-mbed/drivers/",
    "~/.platformio/packages/framework-mbed/events/",
    "~/.platformio/packages/framework-mbed/hal/include/",
    "~/.platformio/packages/framework-mbed/rtos/",
    "~/.platformio/packages/framework-mbed/storage/",
    "~/.platformio/packages/framework-mbed/tools/**",
    "~/.platformio/packages/framework-mbed/targets/TARGET_STM/",
    "~/.platformio/packages/framework-mbed/targets/TARGET_STM/TARGET_STM32F4/",
    "~/.platformio/packages/framework-mbed/targets/TARGET_STM/TARGET_STM32F4/STM32Cube_FW/**",
    "~/.platformio/packages/framework-mbed/targets/TARGET_STM/TARGET_STM32F4/TARGET_STM32F446xE/",
    "~/.platformio/packages/framework-mbed/targets/TARGET_STM/TARGET_STM32F4/TARGET_STM32F446xE/TARGET_NUCLEO_F446RE/**",
    "~/.platformio/packages/framework-mbed/platform/cxxsupport"
]

def main():
    time.sleep(5) #platformioがc_cpp_properties.jsonを書き換えるまで待つ

    #ファイル開く
    with open(".vscode/c_cpp_properties.json", "r") as f:
        text = f.read()
    
    #コメント消す
    while text.startswith('//'):
        _, text = text.split('\n',1)

    #辞書型に直す
    dictionary = json.loads(text)
    
    #includePath書き換える
    dictionary["configurations"][0]["includePath"] = include_path

    #保存
    with open(".vscode/c_cpp_properties.json", "w") as f:
        json.dump(dictionary, f, indent=4)

main()