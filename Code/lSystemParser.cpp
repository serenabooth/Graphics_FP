#include <vector>
#include <string>
#include <memory>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>

#define MAX(a, b) ((a > b) ? a : b) 

using namespace std; 

std::map<std::string,std::string> rules;
std::string axiom; 
int longest_rule = 0; 

void readLSystem() 
{ 
    std::ifstream file("l1.txt");
    std::string str; 

    int i = 0; 
    while (std::getline(file, str))
    {
        if (i == 0) {
            axiom = str;  
        }
        else {
            rules[str.substr(0, str.find(","))] = str.substr(str.find(", ") + 2); 
            // cout << "Added a rule" << endl; 
            // cout << "LHS" << str.substr(0, str.find(",")) << endl; 
            // cout << "RHS" << str.substr(str.find(", ") + 2) << endl; 
            longest_rule = MAX(str.substr(0, str.find(",")).length(), longest_rule); 
        }
        //cout << str << endl; 
        //cout << longest_rule << endl; 

        i++; 
    }
}


std::string gen_string(std::string last_string, int iter, int final_iter) {
    if (iter == final_iter) {
        return last_string; 
    }

    int i = 0; 
    std::string output = ""; 

    while (i < last_string.length()) {
        bool fl = false; 
        for (int j = 1; j <= longest_rule; ++j) {
            if ( j > last_string.length()) {
                break; 
            } 

            std::map<std::string, std::string>::iterator it = rules.find(last_string.substr(i,j));

            if (it != rules.end()) {
                output += it->second; 

                i += j; 
                fl = true; 
                break; 
            }
        }
        if (!fl) {
            output += last_string.substr(i,1);
            i += 1; 
        }
    }
    // if (iter >= 2) 
    // cout << "Substring test " << last_string.substr(1,1) << endl;
    // cout << output << endl;
    // cout << iter << endl;  

    return gen_string(output, iter + 1, final_iter); 
}

int main() {
    readLSystem(); 
    cout << gen_string(axiom, 0, 3) << endl; 
}