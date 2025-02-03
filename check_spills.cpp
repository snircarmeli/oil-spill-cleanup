// Check spills in the folder "spills"

#include "oil-spill.h"
#include <iostream>
#include <fstream>
#include <filesystem> // For file operations

using namespace std::filesystem;
using std::cout;
using std::endl;
using std::string;
using std::ifstream;

int main() {
    // Check if the spills folder exists
    if (!std::filesystem::exists("spills")) {
        cout << "Folder spills does not exist." << endl;
        return 1;
    }

    // Check if the spills folder is a directory
    if (!std::filesystem::is_directory("spills")) {
        cout << "spills is not a directory." << endl;
        return 1;
    }

    // Check if the spills folder is empty
    if (std::filesystem::is_empty("spills")) {
        cout << "spills is empty." << endl;
        return 1;
    }

    // Check if the spills folder can be opened
    ifstream file("spills");
    if (!file.is_open()) {
        cout << "Could not open spills." << endl;
        return 1;
    }

    // Check if the spills folder can be closed
    file.close();
    if (file.is_open()) {
        cout << "Could not close spills." << endl;
        return 1;
    }

    // Iterate over the files in the spills folder
    for (const auto &entry : std::filesystem::directory_iterator("spills")) {
        // Check if the file is a regular file
        if (entry.is_regular_file()) {
            string filename = entry.path().string();
            try {
                OilSpill oil_spill(filename);
                if (!oil_spill.is_valid_sequence()) {
                    cout << "Invalid sequence of points in file: " << filename << endl;
                    cout << endl;
                    cout.flush();
                }
            } catch (const std::exception &e) {
                cout << "Error processing file: " << filename << ". Error: " << e.what() << endl;
                cout << endl;
                cout.flush();
                // Delete the file
                if (!std::filesystem::remove(filename)) {
                    cout << "Failed to delete file: " << filename << endl;
                    cout << endl;
                    cout.flush();
                }
            }
        }
    }
}