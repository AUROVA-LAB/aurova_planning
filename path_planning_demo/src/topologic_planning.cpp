#include "topologic_planning.h"

TopologicPlanning::TopologicPlanning(void)
{
  this->st_path_.num_points = 0;
}

TopologicPlanning::~TopologicPlanning(void)
{
  int i;

  //free memory of goals
  delete[] this->st_goals_;

  //free memory of nodes
  for (i = 1; i < this->st_nodes_[1].num_nodes; i++)
  {
    delete[] this->st_nodes_[i].id_neighbors;
    delete[] this->st_nodes_[i].index_links;
    delete[] this->st_nodes_[i].sense_links;
  }
  delete[] this->st_nodes_;

  //free memory of links
  for (i = 1; i < this->st_links_[1].num_links; i++)
  {
    delete[] this->st_links_[i].points_x;
    delete[] this->st_links_[i].points_y;
    delete[] this->st_links_[i].point_id;
  }
  delete[] this->st_links_;

  //free memory of path
  if (this->st_path_.num_points > 0)
  {
    this->st_path_.num_points = 0;
    delete[] this->st_path_.points_x;
    delete[] this->st_path_.points_y;
    delete[] this->st_path_.point_id;
  }
}

///////////////////////////////////////////////////////////////
// TopoligicPlanning Public API
///////////////////////////////////////////////////////////////

int TopologicPlanning::generateSecPathFromBox(void)
{
  int i, j;
  int counter = 0;

  //check if path is active
  if (this->st_path_.num_points > 0)
  {
    this->st_path_.num_points = 0;
    delete[] this->st_path_.points_x;
    delete[] this->st_path_.points_y;
    delete[] this->st_path_.point_id;
  }

  //get number of points, and reserve memory for path
  for (i = 1; i < this->st_links_[1].num_links; i++)
  {
    this->st_path_.num_points = this->st_path_.num_points + this->st_links_[i].num_points;
  }
  this->st_path_.points_x = new float[this->st_path_.num_points];
  this->st_path_.points_y = new float[this->st_path_.num_points];
  this->st_path_.point_id = new int[this->st_path_.num_points];

  //complete path
  for (i = 1; i < this->st_links_[1].num_links; i++)
  {
    for (j = 0; j < this->st_links_[i].num_points; j++)
    {
      this->st_path_.points_x[counter] = this->st_links_[i].points_x[j];
      this->st_path_.points_y[counter] = this->st_links_[i].points_y[j];
      this->st_path_.point_id[counter] = this->st_links_[i].point_id[j];
      counter++;
    }
  }

  return 0;
}

int TopologicPlanning::loadLinksFromFile(std::string filePath)
{
  std::ifstream file;
  std::string line;
  float point = 0.0;
  int num_of_links = 0;
  int index_link = 0;
  int index_point_id = 0;
  int i, j;

  //openning links file
  file.open(filePath.c_str());
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
    this->st_links_[index_link].point_id = new int[this->st_links_[index_link].num_points];

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
      //add id to each point.
      this->st_links_[index_link].point_id[j] = index_point_id;
      index_point_id++;
    }
  }

  file.close();

  return 0;
}

int TopologicPlanning::loadNodesFromFile(std::string filePath)
{
  std::ifstream file;
  std::string line;
  int num_of_nodes = 0;
  int index_nodes = 0;
  int i, j, k;

  //openning nodes file
  file.open(filePath.c_str());

  //get number of nodes
  if (std::getline(file, line))
  {
    sscanf(line.c_str(), "%d", &num_of_nodes);
  }

  //reserve memory for nodes vector. +1 is for index 1 to n = num_of_nodes
  this->st_nodes_ = new struct Nodes[num_of_nodes + 1];

  //get info about nodes
  for (i = 0; i < num_of_nodes; i++)
  {
    //get id, poses, and number of neighbors.
    index_nodes++;
    this->st_nodes_[index_nodes].num_nodes = num_of_nodes + 1;
    if (std::getline(file, line))
    {
      sscanf(line.c_str(), "%d", &this->st_nodes_[index_nodes].id_node);
    }
    if (std::getline(file, line))
    {
      sscanf(line.c_str(), "%f", &this->st_nodes_[index_nodes].pose_x);
    }
    if (std::getline(file, line))
    {
      sscanf(line.c_str(), "%f", &this->st_nodes_[index_nodes].pose_y);
    }
    if (std::getline(file, line))
    {
      sscanf(line.c_str(), "%d", &this->st_nodes_[index_nodes].num_neighbors);
    }

    //reserve memory for neighbors information
    this->st_nodes_[index_nodes].id_neighbors = new int[this->st_nodes_[index_nodes].num_neighbors];
    this->st_nodes_[index_nodes].index_links = new int[this->st_nodes_[index_nodes].num_neighbors];
    this->st_nodes_[index_nodes].sense_links = new int[this->st_nodes_[index_nodes].num_neighbors];

    //get neighbors information
    for (j = 0; j < this->st_nodes_[index_nodes].num_neighbors; j++)
    {
      if (std::getline(file, line))
      {
        sscanf(line.c_str(), "%d", &this->st_nodes_[index_nodes].id_neighbors[j]);
      }

      //search of links (and sense) joining with neighboring nodes
      for (k = 1; k < this->st_links_[1].num_links; k++)
      {
        if (this->st_links_[k].id_link[0] == this->st_nodes_[index_nodes].id_neighbors[j])
        {
          this->st_nodes_[index_nodes].index_links[j] = k;
          this->st_nodes_[index_nodes].sense_links[j] = INVERSE_SENSE;
          k = this->st_links_[1].num_links + 1;
        }
        else if (this->st_links_[k].id_link[1] == this->st_nodes_[index_nodes].id_neighbors[j])
        {
          this->st_nodes_[index_nodes].index_links[j] = k;
          this->st_nodes_[index_nodes].sense_links[j] = DIRECT_SENSE;
          k = this->st_links_[1].num_links + 1;
        }
      }
    }
  }

  file.close();

  return 0;
}

int TopologicPlanning::loadGoalsFromFile(std::string filePath)
{
  std::ifstream file;
  std::string line;
  int num_of_goals = 0;
  int index_goals = 0;
  int id_link_0, id_link_1;
  int i, j;

  //openning nodes file
  file.open(filePath.c_str());

  //get number of goals
  if (std::getline(file, line))
  {
    sscanf(line.c_str(), "%d", &num_of_goals);
  }

  //reserve memory for goals vector. +1 is for index 1 to n = num_of_goals
  this->st_goals_ = new struct Goals[num_of_goals + 1];

  //get info about goals
  for (i = 0; i < num_of_goals; i++)
  {
    //get id, and poses of goals
    index_goals++;
    this->st_goals_[index_goals].num_goals = num_of_goals + 1;
    if (std::getline(file, line))
    {
      sscanf(line.c_str(), "%d", &this->st_goals_[index_goals].id_goal);
    }
    if (std::getline(file, line))
    {
      sscanf(line.c_str(), "%f", &this->st_goals_[index_goals].pose_x);
    }
    if (std::getline(file, line))
    {
      sscanf(line.c_str(), "%f", &this->st_goals_[index_goals].pose_y);
    }
    if (std::getline(file, line))
    {
      sscanf(line.c_str(), "%d", &id_link_0);
    }
    if (std::getline(file, line))
    {
      sscanf(line.c_str(), "%d", &id_link_1);
    }

    //search for the link where the goal is located
    for (j = 1; j < this->st_links_[1].num_links; j++)
    {
      if (this->st_links_[j].id_link[0] == id_link_0 & this->st_links_[j].id_link[1] == id_link_1)
      {
        this->st_goals_[index_goals].index_link = j;
        j = this->st_links_[1].num_links + 1;
      }
    }
  }

  return 0;
}
