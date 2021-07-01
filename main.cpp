#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

using namespace std;

bool load_transformation_matrix_from_file(
    const std::string &file_name, Eigen::Matrix4d *transformation_matrix,string *parentf,string *childf) {
  //cout << "load_transformation_matrix_from_file is called; file name: "<< file_name << endl;
  try {
    YAML::Node config = YAML::LoadFile(file_name);
    if (!config) {
      cout << "[WARN] Open TransformationMatrix File: " << file_name << "failed." << endl;
      return false;
    }
    if (!config["transform"]) {
      cout << "Open TransformationMatrix File: " << file_name << "has no transform." << endl;
      return false;
    }
    //fill frame_ids
    if(config["child_frame_id"]){
        (*childf) = config["child_frame_id"].as<string>();
    }
    if(config["header"]["frame_id"]){
        (*parentf) = config["header"]["frame_id"].as<string>();
    }

    //fill translation
    if (config["transform"]["translation"]) {
      (*transformation_matrix)(0, 3) =
          config["transform"]["translation"]["x"].as<double>();
      (*transformation_matrix)(1, 3) =
          config["transform"]["translation"]["y"].as<double>();
      (*transformation_matrix)(2, 3) =
          config["transform"]["translation"]["z"].as<double>();
    } else {
      cout << "TransformationMatrix File: " << file_name << "has no transform:translation." << endl;
      return false;
    }
    // fill rotation
    if (config["transform"]["rotation"]) {
      double qx = config["transform"]["rotation"]["x"].as<double>();
      double qy = config["transform"]["rotation"]["y"].as<double>();
      double qz = config["transform"]["rotation"]["z"].as<double>();
      double qw = config["transform"]["rotation"]["w"].as<double>();
      Eigen::Quaternion<double> rotation(qw, qx, qy, qz);
      (*transformation_matrix).block<3, 3>(0, 0) = rotation.toRotationMatrix();
    } else {
      cout << "TransformationMatrix File: " << file_name << " has no transform:rotation." << endl;
      return false;
    }
  } catch (const YAML::Exception &e) {
    cout <<"[Error]" << file_name << "load failed. error: " << e.what();
    cout << "Please ensure param file is exist or format is correct" << endl;
    return false;
  }

  // fill trivial elements
  for (int i = 0; i < 3; i++) {
    (*transformation_matrix)(3, i) = 0.0;
  }
  (*transformation_matrix)(3, 3) = 1.0;
  return true;
}

int save_transformation_to_file(const string file_name, Eigen::Matrix4d *transformation_matrix,const string parentf,const string childf){
    Eigen::Matrix3d rotation_matrix = (*transformation_matrix).block(0,0,3,3);
    Eigen::Quaterniond eigen_quat(rotation_matrix);
    Eigen::Vector4d q_vector = eigen_quat.coeffs();
    ofstream fout(file_name.c_str());
    //create Nodes 
    YAML::Node rootNode;
    rootNode["child_frame_id"] = childf.c_str();
    rootNode["header"]["frame_id"] = parentf.c_str();
    rootNode["transform"]["translation"]["x"] = (*transformation_matrix)(0,3);
    rootNode["transform"]["translation"]["y"] = (*transformation_matrix)(1,3);
    rootNode["transform"]["translation"]["z"] = (*transformation_matrix)(2,3);
    std::cout << "translation: " << (*transformation_matrix)(0,3) << "," << 
                                    (*transformation_matrix)(1,3) << "," << 
                                    (*transformation_matrix)(2,3) << "," << std::endl;
    rootNode["transform"]["rotation"]["x"] = q_vector(0);
    rootNode["transform"]["rotation"]["y"] = q_vector(1);
    rootNode["transform"]["rotation"]["z"] = q_vector(2);
    rootNode["transform"]["rotation"]["w"] = q_vector(3);

    std::cout << "quaternion: x: " << q_vector(0) << "," <<
                "quaternion: y: " << q_vector(1) << "," <<
                "quaternion: z: " << q_vector(2) << "," <<
                "quaternion: w: " << q_vector(3) << "," << std::endl;
    fout << rootNode;
    fout.close();
}


int main(int argc, char *argv[]){
    cout << "Please tell me the PARENT yaml, CHILD yaml and I will generate the result for you." << endl;
    for (int i = 0; i < argc; ++i){
         cout << argv[i] << endl;
    }
    if(argc!=4){
        cout << "[Error] Please check your input." << endl;
        return 0;
    }

    string parent_file = argv[1];
    string child_file = argv[2];
    string output_file = argv[3];
    cout << "parent file: " << parent_file << endl;
    cout << "child file: " << child_file << endl;
    cout << "output file: " << output_file << endl;
    Eigen::Matrix4d *parent_matrix = new Eigen::Matrix4d;
    Eigen::Matrix4d *child_matirx = new Eigen::Matrix4d;

    string parent1f;
    string child1f;
    string parent2f;
    string child2f;

    bool ret1 = load_transformation_matrix_from_file(parent_file,parent_matrix,&parent1f,&child1f);
    bool ret2 = load_transformation_matrix_from_file(child_file,child_matirx,&parent2f,&child2f);
    if(!(ret1 && ret2)){
        cout << "Load yaml fialed!" << endl;
        return -1;
    }
    cout << "parent1f: " << parent1f << endl;
    cout << "child2f: " << child2f << endl;
    if(child1f != parent2f){
        cout << "Can not do transformation because 1 file child is not the same with 2 file parent." << endl;
        return -1;
    }
    cout << "parent matrix: " << (*parent_matrix)(0,3) << "," << 
                                    (*parent_matrix)(1,3) << "," << 
                                    (*parent_matrix)(2,3) << "," << endl;
    cout << "child matrix: " << (*child_matirx)(0,3) << "," << 
                                    (*child_matirx)(1,3) << "," << 
                                    (*child_matirx)(2,3) << "," << endl;
    Eigen::Matrix4d *result_matrix = new Eigen::Matrix4d;
    *result_matrix = (*parent_matrix) * (*child_matirx);
    cout << "result matrix: " << (*result_matrix)(0,3) << "," << 
                                    (*result_matrix)(1,3) << "," << 
                                    (*result_matrix)(2,3) << "," << endl;
    
    save_transformation_to_file(output_file,result_matrix,parent1f,child2f);

    return 0;
}