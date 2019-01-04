#include "toollib.h"

#define __START_LINE 60
#define __END_LINE 110

/** 
 * @brief  Read config file,   
 * @param 
 * @param    
 *
 * @return <0==failed   0==ok
 *     
 */
int read_config(const std::string file, std::vector<std::string>& v)
{   

    fstream f(file.c_str());
    string      line; 
	uint32_t line_count=0;
    while(getline(f,line)) 
    {
		++line_count;
		if (line_count>=__START_LINE && line_count<__END_LINE)
		{/*must end */
//	printf("%s\n",line.c_str()[0]);
			if(line.size()>=3 && line=="end")
				break;
			else
            v.push_back(line); 
        }		
    }
   
    return   v.size();


}

/** 
 * @brief  Write config file,   
 * @param 
 * @param    
 *
 * @return <0==failed   0==ok
 *     
 */
int write_config(const std::string file,uint32_t ops,string will_value)
{
    ofstream openfile(file.c_str(),std::ios::app);
	openfile.seekp(ops, std::ios::end);

	openfile <<"static_lease "<< will_value<<endl;
	openfile.close();

	return 0;
}


/** 
 * @brief  Create a folder and check if it already exists.
 * @param 
 * @param    
 *
 * @return <0==failed   0==ok
 *     
 */
 
int creat_dir(const std::string path)
{
	/*not found*/
	if(access(path.c_str(),F_OK)==-1)   
    {    
      if( !mkdir(path.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO) )
	  {
         return 0;
	  }
      else
        return -1;
    } 
	
	return 0;
}

/*

   Thread function hook, mainly realize thread detach.

 */
 void __float_to_string(float fnum,string & result)
{
	stringstream ss;
	ss << fnum << flush;
	result= ss.str();
	return ;
}

/*

   Thread function hook, mainly realize thread detach.

 */
 void __string_to_float(string strnum,float *result)
{
	float tmp;
	stringstream ss(strnum);
	ss >> tmp;
	*result =tmp;
	return  ;
}

 /** 
 * @brief   Xor check
 * @param Prepare the fields to be encrypted
 *
 * @return Encryption result
 *     
 */
string __xor_send_check(string data)
{
	char  result[4];
	const char* p = data.data();
	uint8_t start=p[0]&0xff;
	uint32_t len=strlen(p);

	if (!len)
	{
		printf("xor pack error\n");
		return"";
	}

	for (uint32_t i=1;i<len ;i++)
	{
		start^=p[i];
	}

	sprintf(result,"%x", start);

	string ret=result;
	return ret;
}

 /** 
 * @brief   Get the current system time,
                 pay attention to your system time is correct
 *
 * @return  string time
 *     
 */
string get_time()
{
	time_t timep;
	time (&timep);
	char tmp[64];
	strftime(tmp, sizeof(tmp), "%Y%m%d%H%M%S",localtime(&timep) );
	return tmp;
}

