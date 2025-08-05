// check_obstacles.cpp
//
// Scan every file in "obstacles/", construct an Obstacle object,
// let Obstacle::is_valid_sequence() judge geometry, and:
//
//   • good  →  save convex hull to obstacles_convexed/.
//   • bad   →  print diagnostics and delete the offending file.
//
// Build:
// g++ -std=c++17 check_obstacles.cpp obstacle.cpp helper_funcs.cpp -I/usr/include/json -I/usr/include/eigen3 -o check_obstacles.exe
// ------------------------------------------------------------------

#include "obstacle.h"
#include <filesystem>
#include <iostream>
#include <fstream>

namespace fs = std::filesystem;
using std::cout;
using std::endl;
using std::string;

int main()
{
    const string in_folder  = "obstacles";
    const string out_folder = "obstacles_convexed";
    fs::create_directory(out_folder);          // silently succeeds if exists

    if (!fs::exists(in_folder) || !fs::is_directory(in_folder)) {
        cout << "Folder '" << in_folder << "' is missing.\n";
        return 1;
    }
    if (fs::is_empty(in_folder)) {
        cout << "'" << in_folder << "' is empty.\n";
        return 1;
    }

    for (const auto& entry : fs::directory_iterator(in_folder))
    {
        if (!entry.is_regular_file()) continue;
        const string fname = entry.path().string();

        try {
            Obstacle obs(fname);                      // may throw on bad header
            cout.flush();
            if (obs.is_valid_sequence())             // geometric check
            {
                // file stem without extension
                string stem = entry.path().stem().string() + "_convex";
                obs.print_convex_hull_to_file(out_folder, stem);
                cout << "Saved spill " << fname << " to " << out_folder << "/" << stem << ".txt" << endl;
                cout.flush();
            }
            else {
                cout << "Invalid geometry in " << fname << endl;
                cout.flush();
                // Delete the file
                if (!fs::remove(fname)) {
                    cout << "Failed to delete file: " << fname << endl;
                } else {
                    cout << "Deleted file: " << fname << endl;
                }
            }
        }
        catch (const std::exception& e)
        {
            cout << "Error processing " << fname << ": " << e.what() << endl;
            // Delete the file
            if (!fs::remove(fname)) {
                cout << "Failed to delete file: " << fname << endl;
            } else {
                cout << "Deleted file: " << fname << endl;
            }
        }
    }
    return 0;
}
