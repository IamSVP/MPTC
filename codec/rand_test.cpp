#include <iostream>
#include <cstdlib>
#include <fstream>




int main(int argc, char *argv[]) {
  uint32_t a = 10;
  std::ofstream out("blah.bin", std::ios::binary);

  out.write(reinterpret_cast<const char*>(&a), 4);
  a = 20;
  out.write(reinterpret_cast<const char*>(&a), 4);
  a = 30;
  out.write(reinterpret_cast<const char*>(&a), 4);
  out.flush();
  a = 40;
  out.write(reinterpret_cast<const char*>(&a), 4);

  out.seekp(0, out.beg);
  out.seekp(4);
  a = 100;
  out.write(reinterpret_cast<const char*>(&a), 4);
  out.close();

  std::ifstream in("blah.bin", std::ios::binary);
  uint32_t b;
  in.read(reinterpret_cast<char*>(&b), 4);
  std::cout << b << std::endl;
  in.read(reinterpret_cast<char*>(&b), 4);
  std::cout << b << std::endl;
  in.read(reinterpret_cast<char*>(&b), 4);
  std::cout << b << std::endl;
  in.read(reinterpret_cast<char*>(&b), 4);
  std::cout << b << std::endl;

  
  return 0;
}
