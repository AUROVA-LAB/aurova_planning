#include "topologic_planning.h"

TopologicPlanning::TopologicPlanning(void)
{

}

TopologicPlanning::~TopologicPlanning(void)
{
  int i;

  for (i = 1; i < this->st_links_[1].num_links; i++)
  {
    delete[] this->st_links_[i].points_x;
    delete[] this->st_links_[i].points_y;
  }
  delete[] this->st_links_;
}

///////////////////////////////////////////////////////////////
// TopoligicPlanning Public API
///////////////////////////////////////////////////////////////

int TopologicPlanning::loadLinksFromFile(char *filePath)
{
  std::ifstream file;
  std::string line;
  float point = 0.0;
  int num_of_links = 0;
  int index_link = 0;
  int i, j;

  //openning links file
  file.open(filePath);
  //assert(file.is_open() && "Error: in file lecture. TopologicPlanning::loadLinksFromFile");

  //get number of links
  if (std::getline(file, line))
  {
    sscanf(line.c_str(), "%d", &num_of_links);
  }

  //reserve memory for links vector. +1 is for index 1 to n = num_of_links
  this->st_links_ = new struct Links[num_of_links + 1];

  //get info about links
  for (i = 0; i < num_of_links; i++)
  {
    //get id and number of points of link
    index_link++;
    this->st_links_[index_link].num_links = num_of_links + 1;
    if (std::getline(file, line))
    {
      sscanf(line.c_str(), "%d", &this->st_links_[index_link].id_link[0]);
    }
    if (std::getline(file, line))
    {
      sscanf(line.c_str(), "%d", &this->st_links_[index_link].id_link[1]);
    }
    if (std::getline(file, line))
    {
      sscanf(line.c_str(), "%d", &this->st_links_[index_link].num_points);
    }

    //reserve memory of points vector
    this->st_links_[index_link].points_x = new float[this->st_links_[index_link].num_points];
    this->st_links_[index_link].points_y = new float[this->st_links_[index_link].num_points];

    //get points of link
    for (j = 0; j < this->st_links_[index_link].num_points; j++)
    {
      if (std::getline(file, line))
      {
        sscanf(line.c_str(), "%f", &point);
        this->st_links_[index_link].points_x[j] = point;
      }
    }
    for (j = 0; j < this->st_links_[index_link].num_points; j++)
    {
      if (std::getline(file, line))
      {
        sscanf(line.c_str(), "%f", &point);
        this->st_links_[index_link].points_y[j] = point;
      }
    }
  }

  file.close();

  return 0;
}
