// Peter Milder
// Testbench Generator for Project 3, ESE 507 Fall 2021
// This code may not be redistributed

#include <iostream>
#include <fstream>
#include <vector>
#include <time.h>
#include <cstdlib>
#include <cstdio>
#include <bitset>

using namespace std;


// Function prototypes
void printUsage();
void genRandomVector(vector<long>& v, int size, int bits);
void computeOutputs(vector<long>& B, vector<long>& x, vector<long>& y, int N, int M, int T);
long saturate(long v, int bits);
string hexString(long v, int bits);



int main(int argc, char* argv[]) {

    // initialize random number generator
    srand(time(NULL)); 

    // If enough input parameters are not provided, print a message and quit.
    if (argc < 5) {
        printUsage();
        return 1;
    }

    // The testbench will generate approximately this many test input values.
    // Why is this approximate? Because the inputs must be in groups of N.
    int approxInputs = 10000;
    int numInputs;   // will set below after we determine N

    // Determine the mode from the command line parameter
    int mode = atoi(argv[1]);

    // For managing file and module names
    string constFileName, inFileName, expResFileName, dut_name, tb_name, out_file;
    ofstream os;

    // Parameter T (number of bits)
    int T;

    // For storing the expected outputs
    vector<long> outputVals;
    

    // -------------------------------------------------------------------------
    // Modes 1 and 2: testbench for a single convolution
    if (((mode == 1) && (argc == 5)) || ((mode == 2) && (argc == 6))) {

        // Read parameters from command line
        int N = atoi(argv[2]);
        int M = atoi(argv[3]);
        T = atoi(argv[4]);

        int P;
        if (mode == 1) 
            P = 1;
        else 
            P = atoi(argv[5]);


        // Calculate the exact number of inputs given approxInputs and N
        numInputs = approxInputs - (approxInputs%N);
        if (numInputs < N) {
            cout << "ERROR: N > numInputs" << endl;
            return 1;
        }

        // The module name of the top-level design (DUT)
        dut_name = "conv_" + to_string(N) + "_" + to_string(M) + "_" + to_string(T) + "_" + to_string(P);

        // Step 1: Generate M random values (for f vector).
        // Store them in the vector here, and also output them to a file.
        // We are going to generate all constants to use only (T/2)+1 bits in order to 
        // make saturations relatively rare.
        vector<long> fVector;
        genRandomVector(fVector, M, (T/2)+1);

        // Store the f constants to a file named const_N_M_T_P.txt
        constFileName = "const_" + to_string(N) + "_" + to_string(M) + "_" + to_string(T) + "_" + to_string(P) + ".txt";
        os.open(constFileName);
        if (!os.is_open()) {
            cout << "ERROR opening " << constFileName << " for writing." << endl;
            return 1;
        }
        for (int i=0; i<M; i++) {
            os << fVector[i] << endl;
        }
        os.close();

        // Step 2: Generate random input values x. 
        // Similarly, we will store them to a vector and output them to a file.
        // Again, we will make these inputs (T/2)+1 bits to reduce the probability
        // of saturation.
        vector<long> inputVals;
        genRandomVector(inputVals, numInputs, (T/2)+1);

        // Store the input values x to a file called tb_conv_N_M_T_P.in
        inFileName = "tb_conv_" + to_string(N) + "_" + to_string(M) + "_" + to_string(T) + "_" + to_string(P) + ".in";
        os.open(inFileName);
        if (!os.is_open()) {
            cout << "ERROR opening " << inFileName << " for writing." << endl;
            return 1;
        }
        for (int i=0; i<numInputs; i++) {
            string s = bitset<32>(inputVals[i]).to_string(); 
            os << s.substr(32-T, 32) << endl;
        }
        os.close();

        // Step 3: Compute the expected outputs and store them in a file.
        computeOutputs(fVector, inputVals, outputVals, N, M, T);

        // Store the expected outputs to a file called tb_conv_N_M_T_P.exp
        expResFileName = "tb_conv_" + to_string(N) + "_" + to_string(M) + "_" + to_string(T) + "_" + to_string(P) + ".exp";
        os.open(expResFileName);
        if (!os.is_open()) {
            cout << "ERROR opening " << expResFileName << " for writing." << endl;
            return 1;
        }
        for (int i=0; i<outputVals.size(); i++) {
            string s = bitset<32>(outputVals[i]).to_string();         
            os << s.substr(32-T, 32) << endl;
        }
        os.close();

    }

    // Mode 3: Generate a testbench for a network of three convolutions. 
    else if ((mode == 3) && (argc == 8)) {

        // Read parameters from command line
        int N  = atoi(argv[2]);
        int M1 = atoi(argv[3]);
        int M2 = atoi(argv[4]);
        int M3 = atoi(argv[5]);
        T = atoi(argv[6]);
        int mult_budget = atoi(argv[7]);

        // Calculate the exact number of inputs given approxInputs and N
        numInputs = approxInputs - (approxInputs%N);
        if (numInputs < N) {
            cout << "ERROR: N > numInputs" << endl;
            return 1;
        }

        // The module name of the top-level design (DUT)        
        dut_name = "net_" + to_string(N) + "_" + to_string(M1) + "_" + to_string(M2) + "_" + to_string(M3) + "_" + to_string(T) + "_" + to_string(mult_budget);


        // Step 1: Generate three vectors for filters f, of length M1, M2, and M3
        // Store them in vectors and output them to the constant file.
        // Now, since data will go through 3 layers, we are making the filter values
        // use fewer bits to reduce the probability of saturation.

        vector<long>  fVector1, fVector2, fVector3;
        genRandomVector(fVector1, M1, T/4);
        genRandomVector(fVector2, M2, T/4);
        genRandomVector(fVector3, M3, T/4);

        // Store the constants for all three layers to a file named const_N_M1_M2_M3_T_A.txt
        constFileName = "const_" + to_string(N) + "_" + to_string(M1) + "_" + to_string(M2) + "_" + to_string(M3) + "_" + to_string(T) + "_" + to_string(mult_budget) + ".txt";
        os.open(constFileName);
        if (!os.is_open()) {
            cout << "ERROR opening " << constFileName << " for writing." << endl;
            return 1;
        }
        for (int i=0; i<M1; i++) 
            os << fVector1[i] << endl;
        for (int i=0; i<M2; i++) 
            os << fVector2[i] << endl;
        for (int i=0; i<M3; i++) 
            os << fVector3[i] << endl;
        os.close();

        // Step 2: Generate random inputs values x.
        // Similarly, we will store them to a vector and output them to a file.
        // Again, we will make these inputs (T/2)+1 bits to reduce the probability
        // of saturation.
        vector<long> inputVals;
        genRandomVector(inputVals, numInputs, (T/2)+1);

        // Store the input values x to a file called tb_net_N_M1_M2_M3_T_A.in
        inFileName = "tb_net_" + to_string(N) + "_" + to_string(M1) + "_" + to_string(M2) + "_" + to_string(M3) + "_" + to_string(T) + "_" + to_string(mult_budget) + ".in";
        os.open(inFileName);
        if (!os.is_open()) {
            cout << "ERROR opening " << inFileName << " for writing." << endl;
            return 1;
        }
        for (int i=0; i<numInputs; i++) {
            string s = bitset<32>(inputVals[i]).to_string();         
            os << s.substr(32-T, 32) << endl;
        }
        os.close();

        // Step 3: Compute the expected outputs and store them in a file.
        vector<long> intermediateVals1, intermediateVals2;
        int L1 = N-M1+1;
        int L2 = L1-M2+1;

        computeOutputs(fVector1, inputVals,         intermediateVals1, N,  M1, T);
        computeOutputs(fVector2, intermediateVals1, intermediateVals2, L1, M2, T);
        computeOutputs(fVector3, intermediateVals2, outputVals,        L2, M3, T);

        // Store the expected outputs to a file called tb_net_N_M1_M2_M3_T_A.exp
        expResFileName = "tb_net_" + to_string(N) + "_" + to_string(M1) + "_" + to_string(M2) + "_" + to_string(M3) + "_" + to_string(T) + "_" + to_string(mult_budget) + ".exp";
        os.open(expResFileName);
        if (!os.is_open()) {
            cout << "ERROR opening " << expResFileName << " for writing." << endl;
            return 1;
        }
        for (int i=0; i<outputVals.size(); i++) {
            string s = bitset<32>(outputVals[i]).to_string();         
            os << s.substr(32-T, 32) << endl;
        }
        os.close();


    }
    else {
        printUsage();
        return 1;
    }

    // step 4: generate the testbench .sv file
    // To do this, I wrote a template called tbtemplate.txt.
    // This is the .sv file we want but a number of things need to be filled in based on
    // this design. So, we will use the command line tool "sed" to do the substituions.
    // This code could run outside of our program in a shell script, but it's very convenient
    // to include it here.

    // The module name for the testbench
    tb_name = "tb_" + dut_name;

    // The filename to store the testbench in
    out_file = tb_name + ".sv";

    string myCmd = "cat tbtemplate.txt";
    myCmd += "| sed 's/<TBMODULENAME>/" + tb_name + "/g; ";
    myCmd += " s/<NUMBITS>/" + to_string(T)  + "/g;";
    myCmd += " s/<NUMINPUTVALS>/" + to_string(numInputs)  + "/g;";
    myCmd += " s/<NUMOUTPUTVALS>/" + to_string(outputVals.size())  + "/g;";
    myCmd += " s/<INFILENAME>/" +  inFileName + "/g;";
    myCmd += " s/<EXPFILENAME>/" +  expResFileName + "/g;";
    myCmd += " s/<DUTNAME>/" +  dut_name + "/g;";
    myCmd += "' > " + out_file;
    system(myCmd.c_str());
}


// Compute the expected output values 
void computeOutputs(vector<long>& f, vector<long>& x, vector<long>& y, int N, int M, int T) {
    int iterations = x.size()/N;
    int L = N-M+1;
    for (int i=0; i<iterations; i++) {
        for (int n=0; n<L; n++) {
            y.push_back(0);
            for (int k=0; k<M; k++) {
                long prod = saturate(f[k]*x[n+k+i*N], T);
                y[n+i*L] = saturate(y[n+i*L] + prod, T);
            }
            y[n+i*L] = (y[n+i*L] < 0) ? 0 : y[n+i*L]; // ReLU
        }
    }
}

// Check to see if v will saturate, and if so, return the appropriate saturation value
long saturate(long v, int bits) {
    long maxVal = ((long)1<<(bits-1))-1;
    long minVal = -1*((long)1<<(bits-1));
    long ovVal = ((long)1 << bits);
  
    if (v > maxVal) {
        return maxVal;
    }
    if (v < minVal) {
        return minVal;
    }
    return v;
}


// Generate a random vector of given size with the given number of bits
void genRandomVector(vector<long>& v, int size, int bits) {
    for (int i=0; i<size; i++)
        v.push_back(rand()%(1<<bits)-(1<<(bits-1)));     
}

void printUsage() {
    cout << "usage: ./testgen mode ARGS" << endl;
    cout << "   Mode 1: Testbench for Mode 1 designs (Part 1)" << endl;
    cout << "      ./testgen 1 N M T" << endl;

    cout << "   Mode 2: Testbench for Mode 2 designs (Part 2)" << endl;
    cout << "      ./testgen 2 N M T P" << endl;

    cout << "   Mode 3: Testbench for three interconnected layers (Part 3)" << endl;
    cout << "      Arguments: N, M1, M2, M3, T, B (multiplier budget), const_file" << endl;
    cout << "              e.g.: ./testgen 16 4 5 6 7 15 16" << endl << endl;

   cout << "      See project description for explanation of parameters." << endl << endl;
}
