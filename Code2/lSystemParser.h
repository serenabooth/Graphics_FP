#include <vector>
#include <string>
#include <memory>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
using namespace std; 

#define MAX(a, b) ((a > b) ? a : b) 

class LSystem {
private:
    std::map<std::string,std::string> rules;
    int longest_rule; 

    //LSystem() {readLSystem("l1.txt"); longest_rule = 0; }

public: 
    std::string axiom; 

    LSystem(std::string file_name) { 
        cout << file_name << endl; 
        longest_rule = 0; 
        readLSystem(file_name);  
    }

    void readLSystem(std::string file_name);
    std::string gen_string(std::string last_string, int iter, int final_iter);

};

void LSystem::readLSystem(std::string file_name) 
{ 
    cout << file_name << endl; 

    std::ifstream file;
    file.open(file_name.c_str());
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


std::string LSystem::gen_string(std::string last_string, int iter, int final_iter) {
    if (iter == final_iter) {
        //cout << "last iteration: " << last_string << endl; 
        return last_string; 
    }

    int i = 0; 
    std::string output = ""; 

    //cout << "gen string called" << endl; 
    //cout << "longest_rule: " << longest_rule << endl;

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

    return gen_string(output, iter + 1, final_iter); 
}