This code is included in markdown as it isn't needed to run the reconstruction

```cpp
#include <iostream>
#include <string>

using namespace std;

int main()
{
    string s;
    std::cin >> s;
    
    //cout << "posx,posy,posz,q1,q2,q3,q4" << endl;
    
    int inputLineCount = 0;
    
    while (!cin.eof() && inputLineCount < 50) {
        if (s.size() < 2 || (s[0] != 'p' || s[1] != 'o')) {
            cin >> s;
            continue;
        }
        
        char letter;
        cin >> letter;
        double a, b, c;
        cin >> a >> b >> c;
        
        cin >> s;
        cin >> s;
        cin >> s;
        
        while (letter != '(') {
            cin >> letter;
        }
        
        double qz, qa, qb, qc;
        cin >> qz >> letter >> qa >> letter >> qb >> letter >> qc;
        cin >> letter;
        
        cout << a << ',' << b << ',' << c << ',' << qz << ',' << qa << ',' << qb << ',' << qc << endl;
        
        inputLineCount++;
        cin >> s;
    }
    

    return 0;
}

```
