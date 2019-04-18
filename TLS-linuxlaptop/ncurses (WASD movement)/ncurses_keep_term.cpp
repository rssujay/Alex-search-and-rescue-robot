#include <ncurses.h>
#include <iostream>

int main(void){
  initscr();
  //noecho();
  int c = getch();
  while(c != 'E'){
    c = getch();
    std::cout << c << std::endl;
  }
  endwin();
  return 0;
}

//Modify ncurses
//Reset
//Get TLS working
// Integrate IR and colour
//LIDAR params
