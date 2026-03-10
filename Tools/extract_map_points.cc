/**
 * Extract 3D map points from an ORB-SLAM3 atlas (.osa) file and save as PLY.
 *
 * Usage:
 *   ./extract_map_points <path_to_vocabulary> <path_to_settings> <atlas_name> <output.ply>
 *
 * The atlas_name should match what was used in System.SaveAtlasToFile
 * (without the .osa extension). The .osa file must be in the current
 * working directory (same as when ORB-SLAM3 saved/loaded it).
 *
 * Example:
 *   cd /home/baselines/monocular/atlas/maps
 *   /root/Packages/ORB_SLAM3/Tools/extract_map_points \
 *       /root/Packages/ORB_SLAM3/Vocabulary/ORBvoc.txt \
 *       /home/baselines/monocular/atlas/configs/left_middle.yaml \
 *       atlas_left_middle \
 *       map_points.ply
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <System.h>
#include <Atlas.h>
#include <MapPoint.h>
#include <KeyFrame.h>
#include <Map.h>

using namespace std;

void SavePLY(const string& filename,
             const vector<Eigen::Vector3f>& points) {
    ofstream f(filename);
    if (!f.is_open()) {
        cerr << "ERROR: Cannot open " << filename << endl;
        return;
    }

    f << "ply" << endl;
    f << "format ascii 1.0" << endl;
    f << "element vertex " << points.size() << endl;
    f << "property float x" << endl;
    f << "property float y" << endl;
    f << "property float z" << endl;
    f << "end_header" << endl;

    f << fixed;
    for (const auto& p : points) {
        f << p.x() << " " << p.y() << " " << p.z() << endl;
    }

    f.close();
    cout << "Saved " << points.size() << " points to " << filename << endl;
}

int main(int argc, char** argv) {
    if (argc != 5) {
        cerr << "Usage: " << argv[0]
             << " <vocabulary> <settings> <atlas_name> <output.ply>"
             << endl;
        return 1;
    }

    string vocFile = argv[1];
    string settingsFile = argv[2];
    string atlasName = argv[3];
    string outputFile = argv[4];

    // Create System in MONOCULAR mode with viewer disabled.
    // The settings file must have System.LoadAtlasFromFile set,
    // OR we rely on the atlas name passed here.
    // We'll create a temporary settings file that loads the atlas.
    cout << "Creating ORB-SLAM3 System to load atlas: " << atlasName << endl;

    // We need to ensure the settings file has the LoadAtlasFromFile set.
    // Read the settings file, check if it has the load directive.
    // If not, create a modified copy.
    string effectiveSettings = settingsFile;

    {
        cv::FileStorage fs(settingsFile, cv::FileStorage::READ);
        cv::FileNode node = fs["System.LoadAtlasFromFile"];
        if (node.empty()) {
            // Create a temp settings file with the atlas load directive
            effectiveSettings = "/tmp/extract_settings.yaml";
            ifstream src(settingsFile);
            ofstream dst(effectiveSettings);
            string line;
            bool inserted = false;
            while (getline(src, line)) {
                dst << line << "\n";
                if (!inserted && line.find("File.version") != string::npos) {
                    dst << "System.LoadAtlasFromFile: \""
                        << atlasName << "\"" << "\n";
                    inserted = true;
                }
            }
            if (!inserted) {
                dst << "System.LoadAtlasFromFile: \""
                    << atlasName << "\"" << "\n";
            }
            src.close();
            dst.close();
            cout << "Wrote temp settings with atlas load to: "
                 << effectiveSettings << endl;
        }
        fs.release();
    }

    ORB_SLAM3::System SLAM(vocFile, effectiveSettings,
                            ORB_SLAM3::System::MONOCULAR, false);

    // Get all map points from all maps in the atlas
    ORB_SLAM3::Atlas* pAtlas = SLAM.GetAtlas();
    vector<ORB_SLAM3::Map*> allMaps = pAtlas->GetAllMaps();

    cout << "Atlas contains " << allMaps.size() << " map(s)" << endl;

    vector<Eigen::Vector3f> allPoints;

    for (size_t i = 0; i < allMaps.size(); i++) {
        vector<ORB_SLAM3::MapPoint*> mapPoints = allMaps[i]->GetAllMapPoints();
        int valid = 0;
        for (auto* pMP : mapPoints) {
            if (pMP && !pMP->isBad()) {
                allPoints.push_back(pMP->GetWorldPos());
                valid++;
            }
        }
        cout << "  Map " << i << ": " << valid << " valid points ("
             << mapPoints.size() << " total)" << endl;
    }

    cout << "Total valid points across all maps: "
         << allPoints.size() << endl;

    SavePLY(outputFile, allPoints);

    SLAM.Shutdown();

    return 0;
}
