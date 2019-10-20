--- Russian version below ---

# Steps to compile:

* download and install Segger Embedded Studio for ARM (SES) - free for Nodric hardware users
* download and unpack into any directory Nordic's ZigBee SDK 3.2.0
* download (clone) this repository into `\examples\zigbee\mluvke\dimmer_rgbw` of unpacked ZigBee SDK. main.c file should be located at `\examples\zigbee\mluvke\dimmer_rgbw\main.c`
* open `\examples\zigbee\mluvke\dimmer_rgbw\mluvke_debug_breadboard\mbr\ses\dimmer_rgbw_mluvke_mbr.emProject` project file in SES
* click on Build => Build dimmer_rgbw_mluvke_mbr menu item inside SES to build project
* hex files for manual firmware download will be available at `\examples\zigbee\mluvke\dimmer_rgbw\mluvke_debug_breadboard\mbr\ses\Output` subdirectories depending on Release/Debug setting and so on
* if you have J-Link connected you can simply click on Build => Build and run or Build => Build and debug menu items.


# Что нужно сделать, чтобы скомпилировать и запустить пример:

* скачать и установить Segger Embedded Studio for ARM (SES) - бесплатно для чипов от Nordic
* скачать и распаковать в любую директорию ZigBee SDK 3.2.0 с сайта Nordic
* скачать (или склонировать) этот репозитарий в директорию `\examples\zigbee\mluvke\dimmer_rgbw` от ZigBee SDK 3 так, чтобы файл main.c был по пути `\examples\zigbee\mluvke\dimmer_rgbw\main.c`
* открыть проект `\examples\zigbee\mluvke\dimmer_rgbw\mluvke_debug_breadboard\mbr\ses\dimmer_rgbw_mluvke_mbr.emProject` внутри SES
* нажать на Build => Build dimmer_rgbw_mluvke_mbr в меню SES для сборки проекта
* hex-файлы для самостоятельной прошивки будут доступны в директории `\examples\zigbee\mluvke\dimmer_rgbw\mluvke_debug_breadboard\mbr\ses\Output` в поддиректориях, соответствующих конфигурации (Release/Debug)
* если подключен дебаггер J-Link, то достаточно нажать в меню на Build => Build and run или Build => Build and debug, чтобы запустить прошивку
