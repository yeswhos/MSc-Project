#include <iterator>
#include <vector>
#include <pnew.cpp>

using namespace std;
typedef vector<int> int_vector;
void setup() {
         Serial.begin(9600);
         
         int_vectorl_vector;
         l_vector.push_back(1);
         l_vector.push_back(2);
         l_vector.push_back(3);
         l_vector.push_back(4);
         l_vector.push_back(5);
         int_vector::iteratorpos = l_vector.begin();
         while(pos!= l_vector.end()){
             // statement
             Serial.print("value:");
              Serial.println(*pos);
             pos ++;
         }
}
void loop() {
}
