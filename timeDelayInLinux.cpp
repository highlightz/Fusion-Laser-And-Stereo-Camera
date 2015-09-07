// In linux
#include <unistd.h>

int main( )
{
    int seconds;
    std::cout << "Enter the time to delay: ";
    while( std::cin >> seconds )
    {
        std::cout << "Start timing!" << std::endl;
        sleep( seconds );
        std::cout << "End timing!" << std::endl;
        std::cout << "Enter the time to delay: ";
    }

    return 0;
}
