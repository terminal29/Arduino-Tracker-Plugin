#ifndef _PRINT_HELPER_H_
#define _PRINT_HELPER_H_

// <3 http://forum.arduino.cc/index.php?topic=46290.0
template<typename T> inline Print &operator <<(Print &obj, T arg) { obj.print(arg); return obj; }
inline Print &operator <<(Print &obj, float arg){obj.print(arg, 4); return obj;}
char endl = '\n';
//

#endif
