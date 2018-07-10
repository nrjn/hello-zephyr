#include <zephyr.h>
#include <misc/printk.h>

#define MY_STACK_SIZE 500
#define  MY_PRIORITY 5

void print_hell(void)
{
    for (;;)
    {
        printk("N Hello PANNI\r\n");
        k_sleep(1000);
    }
}

K_THREAD_DEFINE(print_hello, MY_STACK_SIZE,
                print_hell, NULL, NULL, NULL,
                MY_PRIORITY, 0, K_NO_WAIT);

//void main (void)
//{
//  for (;;)
//  {
//    printk("N Hello PANNI\r\n");
//    k_sleep(1000);
//  }
//}
