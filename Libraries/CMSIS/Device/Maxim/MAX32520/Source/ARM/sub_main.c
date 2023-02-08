
extern void SystemInit(void);
extern void $Super$$main(void);

// This will be executed after the RAM initialization
void $Sub$$main(void)
{
    SystemInit();

    // Call to main function
    $Super$$main();
}
