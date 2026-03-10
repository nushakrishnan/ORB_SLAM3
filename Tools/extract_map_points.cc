/**
 * Extract 3D map points and keyframe poses from an ORB-SLAM3 atlas (.osa)
 * file. Saves map points as PLY and keyframe poses as TXT.
 *
 * Usage:
 *   ./extract_map_points <vocabulary> <settings> <atlas_name> <output_prefix>
 *
 * Outputs:
 *   <output_prefix>_points.ply    - 3D map points (colored by map: red/blue)
 *   <output_prefix>_keyframes.txt - keyframe poses (timestamp tx ty tz qx qy qz qw map_id)
 *
 * The atlas_name should match what was used in System.SaveAtlasToFile
 * (without the .osa extension). The .osa file must be in the current
 * working directory.
 *
 * Example:
 *   cd /home/baselines/monocular/atlas/maps
 *   /root/Packages/ORB_SLAM3/Tools/extract_map_points \
 *       /root/Packages/ORB_SLAM3/Vocabulary/ORBvoc.txt \
 *       /home/baselines/monocular/atlas/configs/left_middle.yaml \
 *       atlas_left_middle \
 *       /home/baselines/monocular/atlas/maps/output
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>

#include <System.h>
#include <Atlas.h>
#include <MapPoint.h>
#include <KeyFrame.h>
#include <Map.h>

using namespace std;

// Colors for different maps (RGB 0-255)
static const int MAP_COLORS[][3] = {
    {255, 0, 0},     // red
    {0, 0, 255},     // blue
    {0, 200, 0},     // green
    {255, 165, 0},   // orange
    {128, 0, 128},   // purple
    {0, 200, 200},   // cyan
    {200, 200, 0},   // yellow
    {200, 0, 200},   // magenta
};
static const int NUM_COLORS = 8;

struct ColoredPoint {
    Eigen::Vector3f pos;
    int r, g, b;
};

void SavePLY(const string& filename,
             const vector<ColoredPoint>& points) {
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
    f << "property uchar red" << endl;
    f << "property uchar green" << endl;
    f << "property uchar blue" << endl;
    f << "end_header" << endl;

    f << fixed;
    for (const auto& p : points) {
        f << p.pos.x() << " " << p.pos.y() << " " << p.pos.z()
          << " " << p.r << " " << p.g << " " << p.b << endl;
    }

    f.close();
    cout << "Saved " << points.size() << " points to " << filename << endl;
}

void SaveKeyframes(const string& filename,
                   const vector<ORB_SLAM3::Map*>& allMaps) {
    ofstream f(filename);
    if (!f.is_open()) {
        cerr << "ERROR: Cannot open " << filename << endl;
        return;
    }

    f << "# timestamp tx ty tz qx qy qz qw map_id" << endl;
    f << fixed;

    int totalKFs = 0;
    for (size_t mapIdx = 0; mapIdx < allMaps.size(); mapIdx++) {
        vector<ORB_SLAM3::KeyFrame*> keyframes =
            allMaps[mapIdx]->GetAllKeyFrames();

        // Sort by timestamp
        sort(keyframes.begin(), keyframes.end(),
             [](ORB_SLAM3::KeyFrame* a, ORB_SLAM3::KeyFrame* b) {
                 return a->mTimeStamp < b->mTimeStamp;
             });

        for (auto* pKF : keyframes) {
            if (!pKF || pKF->isBad()) continue;

            Sophus::SE3f Twc = pKF->GetPoseInverse();
            Eigen::Vector3f t = Twc.translation();
            Eigen::Quaternionf q = Twc.unit_quaternion();

            f << pKF->mTimeStamp << " "
              << t.x() << " " << t.y() << " " << t.z() << " "
              << q.x() << " " << q.y() << " " << q.z() << " "
              << q.w() << " " << mapIdx << endl;

            totalKFs++;
        }

        cout << "  Map " << mapIdx << ": " << keyframes.size()
             << " keyframes" << endl;
    }

    f.close();
    cout << "Saved " << totalKFs << " keyframes to " << filename << endl;
}

int main(int argc, char** argv) {
    if (argc != 5) {
        cerr << "Usage: " << argv[0]
             << " <vocabulary> <settings> <atlas_name> <output_prefix>"
             << endl;
        cerr << "Outputs: <output_prefix>_points.ply, "
             << "<output_prefix>_keyframes.txt" << endl;
        return 1;
    }

    string vocFile = argv[1];
    string settingsFile = argv[2];
    string atlasName = argv[3];
    string outputPrefix = argv[4];

    cout << "Creating ORB-SLAM3 System to load atlas: " << atlasName << endl;

    // Ensure the settings file has the LoadAtlasFromFile directive
    string effectiveSettings = settingsFile;

    {
        cv::FileStorage fs(settingsFile, cv::FileStorage::READ);
        cv::FileNode node = fs["System.LoadAtlasFromFile"];
        if (node.empty()) {
            effectiveSettings = "/tmp/extract_settings.yaml";
            ifstream src(settingsFile);
            ofstream dst(effectiveSettings);
            string line;
            bool inserted = false;
            while (getline(src, line)) {
                dst << line << "\n";
                if (!inserted &&
                    line.find("File.version") != string::npos) {
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

    // Get atlas
    ORB_SLAM3::Atlas* pAtlas = SLAM.GetAtlas();
    vector<ORB_SLAM3::Map*> allMaps = pAtlas->GetAllMaps();

    cout << "Atlas contains " << allMaps.size() << " map(s)" << endl;

    // --- Extract map points (colored by map) ---
    vector<ColoredPoint> allPoints;

    for (size_t i = 0; i < allMaps.size(); i++) {
        int ci = i % NUM_COLORS;
        vector<ORB_SLAM3::MapPoint*> mapPoints =
            allMaps[i]->GetAllMapPoints();
        int valid = 0;
        for (auto* pMP : mapPoints) {
            if (pMP && !pMP->isBad()) {
                ColoredPoint cp;
                cp.pos = pMP->GetWorldPos();
                cp.r = MAP_COLORS[ci][0];
                cp.g = MAP_COLORS[ci][1];
                cp.b = MAP_COLORS[ci][2];
                allPoints.push_back(cp);
                valid++;
            }
        }
        cout << "  Map " << i << ": " << valid << " valid points ("
             << mapPoints.size() << " total)" << endl;
    }

    cout << "Total valid points across all maps: "
         << allPoints.size() << endl;

    // --- Save outputs ---
    string pointsFile = outputPrefix + "_points.ply";
    string kfFile = outputPrefix + "_keyframes.txt";

    SavePLY(pointsFile, allPoints);

    cout << endl << "Keyframes per map:" << endl;
    SaveKeyframes(kfFile, allMaps);

    // Summary
    cout << endl << "=== Summary ===" << endl;
    cout << "Maps: " << allMaps.size() << endl;
    if (allMaps.size() == 1) {
        cout << "Maps were MERGED (loop closure succeeded)" << endl;
    } else {
        cout << "Maps are SEPARATE (no merge detected). "
             << "Points are colored by map in the PLY." << endl;
    }

    SLAM.Shutdown();

    return 0;
}
