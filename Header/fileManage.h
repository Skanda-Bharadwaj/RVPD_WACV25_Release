#ifndef FILEMANAGE_H
#define FILEMANAGE_H

#include <general_include.h>

// check if the directory exist already or it will
void check_if_dir(string directory_name)
{
    if(!fs::exists(directory_name))
    {
        fs::path dir(directory_name);
        if(!boost::filesystem::create_directory(dir))
        {
            cerr<<"Can't create a directory!!!! :"<<directory_name<<endl;
        }
    }
}

void read_subdir(vector<string> & listofdir, string & filepath)
{
    // read all the sub_dir of images to list of dir.
    const char* c_im_dir = filepath.c_str();
    struct dirent *de;
    DIR* dr = opendir(c_im_dir);
    if(dr == NULL)
    {
        cerr<<"Couldn't open the dataset directory!"<<endl;
    }
    while((de = readdir(dr))!=NULL)
    {
        string c_dir = de->d_name;
        if(c_dir != "." && c_dir != ".." && c_dir != ".DS_Store" )
             listofdir.push_back(c_dir);
    }
    closedir(dr);
}



#endif // FILEMANAGE_H
