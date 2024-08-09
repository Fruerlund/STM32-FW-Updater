
# Firmware Updater written in C for STM32

This respository contains a implementation of a firwmare updater written in C based on the STM32 Playlist "Blinky to Bootloader". It utilizes a state machine and acustom packet protocol to send a firwmare update remotely using UART.

**You will find, among other things lack of return checks, different use of returns (int, bools), large source files, repetive code, lack of project structure**

The code is not flawless and does not account for all edge cases.


I hope you find this useful.


## Installation  and usage

The program can be used by using a compiler of your choice or utilizing the makefile

For example:

```bash
make
```
    
## Feedback

If you have any feedback, please reach out to me.


## Acknowledgements

 - [Blinky to bootloader](https://www.youtube.com/watch?v=uQQsDWLRDuI&list=PLP29wDx6QmW7HaCrRydOnxcy8QmW0SNdQ)

