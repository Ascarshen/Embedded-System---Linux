#ifndef __TOOLLIB_H_
#define __TOOLLIB_H_
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <stdint.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sstream>
#define __MAX(a,b) ((a)>(b))?(a):(b) 
using namespace std;


#define __START_LINE 60
#define __END_LINE 110
string get_time();
int read_config(const std::string file, std::vector<std::string>& v);
int write_config(const std::string file,uint32_t ops,string will_value);
void __float_to_string(float fnum,string & result);
void __string_to_float(string strnum,float *result);
int creat_dir(const std::string path);
string __xor_send_check(string data);


#endif
