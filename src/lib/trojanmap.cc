#include "trojanmap.h"

//-----------------------------------------------------
// TODO: Students should implement the following:
//-----------------------------------------------------
/**
 * GetLat: Get the latitude of a Node given its id. If id does not exist, return
 * -1.
 *
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
double TrojanMap::GetLat(const std::string &id)
{
  if (data.find(id) != data.end())
  {
    return data[id].lat;
  }
  return -1;
}

/**
 * GetLon: Get the longitude of a Node given its id. If id does not exist,
 * return -1.
 *
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(const std::string &id)
{
  if (data.find(id) != data.end())
  {
    return data[id].lon;
  }
  return -1;
}

/**
 * GetName: Get the name of a Node given its id. If id does not exist, return
 * "NULL".
 *
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(const std::string &id)
{
  if (data.find(id) != data.end())
  {
    return data[id].name;
  }
  return "NULL";
}

/**
 * GetNeighborIDs: Get the neighbor ids of a Node. If id does not exist, return
 * an empty vector.
 *
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(const std::string &id)
{
  if (data.find(id) != data.end())
  {
    return data[id].neighbors;
  }
  return {};
}

/**
 * GetID: Given a location name, return the id.
 * If the node does not exist, return an empty string.
 * The location name must be unique, which means there is only one node with the name.
 *
 * @param  {std::string} name          : location name
 * @return {std::string}               : id
 */
std::string TrojanMap::GetID(const std::string &name)
{
  std::string res = "";

  for (const auto &pair : data)
  {
    if (pair.second.name == name)
    {
      res = pair.first;
    }
  }

  return res;
}

/**
 * GetPosition: Given a location name, return the position. If id does not
 * exist, return (-1, -1).
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name)
{

  // if extra spaces in beginning
  while (!name.empty() && name[0] == ' ')
  {
    name.erase(name.begin());
  }

  // if extra spaces at end
  while (!name.empty() && name[name.size() - 1] == ' ')
  {
    name.pop_back();
  }
  // convert to lowercase
  for (char &c : name)
  {
    c = tolower(c);
  }

  // traverse all nodes on the map
  for (auto &pair : data)
  {
    const Node &node = pair.second; // retrieve current node

    std::string node_name = node.name;
    for (char &c : node_name)
    {
      c = tolower(c);
    }

    // if we find this node’s name, return its position
    if (node_name == name)
    {
      return {node.lat, node.lon};
    }
  }
  return {-1, -1};
}

/**
 * CalculateEditDistance: Calculate edit distance between two location names
 * @param  {std::string} a          : first string
 * @param  {std::string} b          : second string
 * @return {int}                    : edit distance between two strings
 */

// use dyanmic programming!
int TrojanMap::CalculateEditDistance(std::string a, std::string b)
{
  // length of first and second string
  int lengthA = a.size();
  int lengthB = b.size();

  // create vectors to represent current and previous rows
  std::vector<int> prevRow(lengthB + 1);
  std::vector<int> currRow(lengthB + 1);

  // initialize first row for transforming empty string to b
  for (int col = 0; col <= lengthB; ++col)
  {
    prevRow[col] = col;
  }

  // process each character of first string
  for (int row = 1; row <= lengthA; ++row)
  {
    currRow[0] = row;

    for (int col = 1; col <= lengthB; ++col)
    {
      if (a[row - 1] == b[col - 1]) // if characters match
      {
        currRow[col] = prevRow[col - 1];
      }
      else
      {
        // calculate min number of operations
        currRow[col] = 1 + std::min({
                               currRow[col - 1], // insert a character
                               prevRow[col],     // delete a character
                               prevRow[col - 1]  // replace a character
                           });
      }
    }
    // move to next row
    prevRow = currRow;
  }

  // edit distance
  return prevRow[lengthB];
}

/**
 * FindClosestName: Given a location name, return the name with the smallest edit
 * distance.
 *
 * @param  {std::string} name          : location name
 * @return {std::string} tmp           : the closest name
 */
std::string TrojanMap::FindClosestName(std::string name)
{
  std::string tmp = ""; // Start with a dummy word
  int threshold = 4;    // create a threshold (edge case)

  // convert to lowercase
  for (char &c : name)
  {
    c = tolower(c);
  }

  int minDistance = INT_MAX; // use INT_MAX as the initial "large" distance

  // loop through every map entry
  for (const auto &pair : data)
  {
    std::string currName = pair.second.name; // current name

    // skip empty names
    if (currName.empty())
    {
      continue;
    }

    // Convert the current name to lowercase for case-insensitive comparison
    for (char &c : currName)
    {
      c = tolower(c);
    }

    // calculate edit distance
    int distance = CalculateEditDistance(name, currName);

    // check if this is best match
    if (distance < minDistance)
    {
      minDistance = distance;
      tmp = pair.second.name;
    }
  }

  // edge case (eg. if input "HelloWorld")
  if (minDistance > threshold)
  {
    return "";
  }

  // return name with smallest edit distance
  return tmp;
}

/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix. The function should be case-insensitive.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */
std::vector<std::string> TrojanMap::Autocomplete(std::string name)
{
  std::vector<std::string> results;

  // test case: input empty --> return empty vector
  if (name.empty())
  {
    return results;
  }

  // convert input string to lowercase
  for (size_t i = 0; i < name.size(); ++i)
  {
    name[i] = tolower(name[i]); // found tolower function on geeksforgeeks.org
  }

  // traverse all nodes on the map
  for (const auto &entry : data)
  {
    const Node &node = entry.second;

    // convert node name string to lowercase
    std::string node_name = node.name;
    for (size_t i = 0; i < node_name.size(); ++i)
    {
      node_name[i] = tolower(node_name[i]);
    }

    // if the size of input is greater than the size of node's name, we skip this node
    if (node_name.size() < name.size())
    {
      continue;
    }

    // compare two strings, the input one and the substr of node's name starting at the
    // beginning of name with the same length of input, and check whether they are equal or not
    if (node_name.substr(0, name.size()) == name)
    {
      // if they are, push the node's name into result vector
      results.push_back(node.name);
    }
  }

  return results;
}

/**
 * GetAllCategories: Return all the possible unique location categories, i.e.
 * there should be no duplicates in the output.
 *
 * @return {std::vector<std::string>}  : all unique location categories
 */
std::vector<std::string> TrojanMap::GetAllCategories()
{
  std::vector<std::string> categories; // store categories

  // loop through all nodes in map
  for (const auto &pair : data)
  {
    const Node &node = pair.second; // current place

    // loop through each attribute of location
    for (const auto &attributes : node.attributes)
    {
      // add if not in list (will not add duplicates)
      if (std::find(categories.begin(), categories.end(), attributes) == categories.end())
      {
        categories.push_back(attributes);
      }
    }
  }
  // return list of categories
  return categories;
}

/**
 * GetAllLocationsFromCategory: Return all the locations of the input category (i.e.
 * 'attributes' in data.csv). If there is no location of that category, return
 * (-1, -1). The function should be case-insensitive.
 *
 * @param  {std::string} category         : category name (attribute)
 * @return {std::vector<std::string>}     : ids
 */
std::vector<std::string> TrojanMap::GetAllLocationsFromCategory(
    std::string category)
{
  // res stores ids of matching locations
  std::vector<std::string> res;

  // convert to lowercase
  for (char &c : category)
  {
    c = tolower(c);
  }

  // loop through all nodes in map
  for (const auto &pair : data)
  {
    const Node &node = pair.second; // current node

    // loop through each attribute of location
    for (const auto &attributes : node.attributes)
    {
      // make a copy in lowercase
      std::string lcAttribute = attributes;

      for (char &c : lcAttribute)
      {
        c = tolower(c);
      }

      // if attribute matches category --> add to id
      if (lcAttribute == category)
      {
        res.push_back(node.id);
        break;
      }
    }
  }

  // returns all locations that match that category
  return res;
}

/**
 * GetLocationRegex: Given the regular expression of a location's name, your
 * program should first check whether the regular expression is valid, and if so
 * it returns all locations that match that regular expression.
 *
 * @param  {std::regex} location name      : the regular expression of location
 * names
 * @return {std::vector<std::string>}     : ids
 */
std::vector<std::string> TrojanMap::GetLocationRegex(std::regex location)
{
  // res stores ids of matching locations
  std::vector<std::string> res;

  // loop through all nodes in map
  for (const auto &pair : data)
  {
    const std::string &name = pair.second.name; // location name

    if (std::regex_search(name, location)) // found regex_search function on geeksforgeeks.org
    {
      res.push_back(pair.first); // add id of matching location
    }
  }

  // return all location ids that match that regular expression
  return res;
}

/**
 * CalculateDistance: Get the distance between 2 nodes.
 * We have provided the code for you. Please do not need to change this function.
 * You can use this function to calculate the distance between 2 nodes.
 * The distance is in mile.
 * The distance is calculated using the Haversine formula.
 * https://en.wikipedia.org/wiki/Haversine_formula
 *
 * @param  {std::string} a  : a_id
 * @param  {std::string} b  : b_id
 * @return {double}  : distance in mile
 */
double TrojanMap::CalculateDistance(const std::string &a_id,
                                    const std::string &b_id)
{
  // Do not change this function
  Node a = data[a_id];
  Node b = data[b_id];
  double dlon = (b.lon - a.lon) * M_PI / 180.0;
  double dlat = (b.lat - a.lat) * M_PI / 180.0;
  double p = pow(sin(dlat / 2), 2.0) + cos(a.lat * M_PI / 180.0) *
                                           cos(b.lat * M_PI / 180.0) *
                                           pow(sin(dlon / 2), 2.0);
  double c = 2 * asin(std::min(1.0, sqrt(p)));
  return c * 3961;
}

/**
 * CalculatePathLength: Calculates the total path length for the locations
 * inside the vector.
 * We have provided the code for you. Please do not need to change this function.
 *
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path)
{
  // Do not change this function
  double sum = 0;
  for (int i = 0; i < int(path.size()) - 1; i++)
  {
    sum += CalculateDistance(path[i], path[i + 1]);
  }
  return sum;
}

/**
 * CalculateShortestPath_Dijkstra: Given 2 locations, return the shortest path
 * which is a list of id. Hint: Use priority queue.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Dijkstra(
    std::string location1_name, std::string location2_name)
{
  std::vector<std::string> path; // will store final path

  // get ids of start and goal locations
  std::string start_id = GetID(location1_name);
  std::string goal_id = GetID(location2_name);

  // if location not found --> return empty
  if (start_id.empty() || goal_id.empty())
  {
    return path;
  }

  // if start and goal are the same
  if (start_id == goal_id)
  {
    path.push_back(start_id);
    return path;
  }

  // initialize distances with infinity and set starting point distance to 0
  std::unordered_map<std::string, double> distances;
  std::unordered_map<std::string, std::string> previous; // track path
  const double infinity = std::numeric_limits<double>::max();

  for (const auto &node : data)
  {
    distances[node.first] = infinity;
  }
  distances[start_id] = 0;

  // priority queue to store nodes with distances
  std::priority_queue<std::pair<double, std::string>,
                      std::vector<std::pair<double, std::string>>,
                      std::greater<std::pair<double, std::string>>>
      pq;

  // push starting node into queue
  pq.push(std::make_pair(0, start_id));

  while (!pq.empty())
  {
    // get node with smallest distance
    std::pair<double, std::string> top = pq.top();
    pq.pop();
    double current_dist = top.first;
    std::string current = top.second;

    // stop if goal reached
    if (current == goal_id)
    {
      break;
    }

    // update distances for all neighbors
    for (const auto &neighbor : data[current].neighbors)
    {
      double new_dist = distances[current] + CalculateDistance(current, neighbor);
      if (new_dist < distances[neighbor])
      {
        distances[neighbor] = new_dist;
        previous[neighbor] = current;                // track  previous node
        pq.push(std::make_pair(new_dist, neighbor)); // push updated distance into queue
      }
    }
  }

  // reconstruct path from goal to start
  for (std::string at = goal_id; at != start_id; at = previous[at])
  {
    if (previous.find(at) == previous.end()) // if can't reconstruct the path --> return empty
    {
      return {};
    }
    path.push_back(at);
  }
  path.push_back(start_id);
  std::reverse(path.begin(), path.end()); // reverse the path (so it starts at start)

  return path;
}

/**
 * CalculateShortestPath_Bellman_Ford: Given 2 locations, return the shortest
 * path which is a list of id. Hint: Do the early termination when there is no
 * change on distance.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */
std::vector<std::string> TrojanMap::CalculateShortestPath_Bellman_Ford(
    std::string location1_name, std::string location2_name)
{
  std::vector<std::string> path; // will store final path

  // get ids of start and goal locations
  std::string start_id = GetID(location1_name);
  std::string goal_id = GetID(location2_name);

  // if location not found --> return empty
  if (start_id.empty() || goal_id.empty())
  {
    return path;
  }

  // if start and goal are the same
  if (start_id == goal_id)
  {
    path.push_back(start_id);
    return path;
  }

  // initialize distances with infinity and set starting point distance to 0
  std::unordered_map<std::string, double> distances;
  std::unordered_map<std::string, std::string> previous; // track path
  const double infinity = std::numeric_limits<double>::max();

  for (const auto &node : data)
  {
    distances[node.first] = infinity;
  }
  distances[start_id] = 0;

  // relax edges |V| - 1 times
  int V = data.size(); // total number of nodes
  for (int i = 0; i < V - 1; ++i)
  {
    bool updated = false; // track if any distances were updated

    // loop through all nodes and their neighbors
    for (const auto &node : data)
    {
      std::string u = node.first; // current node
      if (distances[u] == infinity)
        continue; // skip unreachable nodes

      for (const auto &neighbor : node.second.neighbors)
      {
        double new_dist = distances[u] + CalculateDistance(u, neighbor); // calculate distance
        if (new_dist < distances[neighbor])
        {
          distances[neighbor] = new_dist; // update the shortest distance
          previous[neighbor] = u;         // store the previous node
          updated = true;                 // mark that update was made
        }
      }
    }

    // if no distances were updated --> stop early
    if (!updated)
    {
      break;
    }
  }

  // check if goal is reachable
  if (distances[goal_id] == infinity)
  {
    return path; // if goal unreachable --> return empty
  }

  // reconstruct path from goal to start
  for (std::string at = goal_id; at != start_id; at = previous[at])
  {
    if (previous.find(at) == previous.end()) // if no path exists
    {
      return {}; // return empty
    }
    path.push_back(at); // add node to path
  }

  path.push_back(start_id);
  std::reverse(path.begin(), path.end()); // reverse the path (so it starts at start)

  return path;
}

/**
 * Traveling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path,
 *                                                                      for example: {10.3, {{0, 1, 2, 3, 4, 0}, {0, 1, 2, 3, 4, 0}, {0, 4, 3, 2, 1, 0}}},
 *                                                                      where 10.3 is the total distance,
 *                                                                      and the first vector is the path from 0 and travse all the nodes and back to 0,
 *                                                                      and the second vector is the path shorter than the first one,
 *                                                                      and the last vector is the shortest path.
 */
// Please use brute force to implement this function, ie. find all the permutations.
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_Brute_force(
    std::vector<std::string> location_ids)
{
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
}

// Please use backtracking to implement this function
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_Backtracking(
    std::vector<std::string> location_ids)
{
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
}

// Hint: https://en.wikipedia.org/wiki/2-opt
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_2opt(
    std::vector<std::string> location_ids)
{
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
}

// This is optional
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravelingTrojan_3opt(
    std::vector<std::string> location_ids)
{
  std::pair<double, std::vector<std::vector<std::string>>> records;
  return records;
}

/**
 * Given CSV filename, it read and parse locations data from CSV file,
 * and return locations vector for topological sort problem.
 * We have provided the code for you. Please do not need to change this function.
 * Example:
 *   Input: "topologicalsort_locations.csv"
 *   File content:
 *    Name
 *    Ralphs
 *    KFC
 *    Chick-fil-A
 *   Output: ['Ralphs', 'KFC', 'Chick-fil-A']
 * @param  {std::string} locations_filename     : locations_filename
 * @return {std::vector<std::string>}           : locations
 */
std::vector<std::string> TrojanMap::ReadLocationsFromCSVFile(
    std::string locations_filename)
{
  std::vector<std::string> location_names_from_csv;
  std::fstream fin;
  fin.open(locations_filename, std::ios::in);
  std::string line, word;
  getline(fin, line);
  while (getline(fin, word))
  {
    location_names_from_csv.push_back(word);
  }
  fin.close();
  return location_names_from_csv;
}

/**
 * Given CSV filenames, it read and parse dependencise data from CSV file,
 * and return dependencies vector for topological sort problem.
 * We have provided the code for you. Please do not need to change this function.
 * Example:
 *   Input: "topologicalsort_dependencies.csv"
 *   File content:
 *     Source,Destination
 *     Ralphs,Chick-fil-A
 *     Ralphs,KFC
 *     Chick-fil-A,KFC
 *   Output: [['Ralphs', 'Chick-fil-A'], ['Ralphs', 'KFC'], ['Chick-fil-A', 'KFC']]
 * @param  {std::string} dependencies_filename     : dependencies_filename
 * @return {std::vector<std::vector<std::string>>} : dependencies
 */
std::vector<std::vector<std::string>> TrojanMap::ReadDependenciesFromCSVFile(
    std::string dependencies_filename)
{
  std::vector<std::vector<std::string>> dependencies_from_csv;
  std::fstream fin;
  fin.open(dependencies_filename, std::ios::in);
  std::string line, word;
  getline(fin, line);
  while (getline(fin, line))
  {
    std::stringstream s(line);
    std::vector<std::string> dependency;
    while (getline(s, word, ','))
    {
      dependency.push_back(word);
    }
    dependencies_from_csv.push_back(dependency);
  }
  fin.close();
  return dependencies_from_csv;
}

/**
 * DeliveringTrojan: Given a vector of location names, it should return a
 * sorting of nodes that satisfies the given dependencies. If there is no way to
 * do it, return a empty vector.
 *
 * @param  {std::vector<std::string>} locations                     : locations
 * @param  {std::vector<std::vector<std::string>>} dependencies     : prerequisites
 * @return {std::vector<std::string>} results                       : results
 */
std::vector<std::string> TrojanMap::DeliveringTrojan(
    std::vector<std::string> &locations,
    std::vector<std::vector<std::string>> &dependencies)
{
  std::vector<std::string> result;                                 // store feasible route
  std::unordered_map<std::string, std::vector<std::string>> graph; // graph to represent dependencies
  std::unordered_map<std::string, int> tasks;                      // store tasks blocking each location

  // initialize for each location
  for (const auto &location : locations)
  {
    graph[location] = {};
    tasks[location] = 0;
  }

  // build the graph from dependencies
  for (const auto &dependency : dependencies)
  {
    std::string from = dependency[0]; // location that must be visited first
    std::string to = dependency[1];   // destination location
    graph[from].push_back(to);        // add dependency ("to" depends on "from")
    tasks[to]++;                      // increase number of tasks
  }

  // use queue to process nodes with no tasks
  std::queue<std::string> toVisit;
  for (const auto &task : tasks)
  {
    if (task.second == 0) // no tasks pending
    {
      toVisit.push(task.first); // add to queue
    }
  }

  // topological sorting
  while (!toVisit.empty())
  {
    std::string currLocation = toVisit.front(); // get next location
    toVisit.pop();
    result.push_back(currLocation); // add location to result

    // process location dependent on current location
    for (const auto &dependent : graph[currLocation])
    {
      tasks[dependent]--; // reduce number of tasks
      if (tasks[dependent] == 0)
      {
        toVisit.push(dependent);
      }
    }
  }

  // check for cycles
  if (result.size() != locations.size())
  {

    return {};
  }

  // return to Tommy the feasible route!
  return result;
}

/**
 * inSquare: Give a id return whether it is in square or not.
 *
 * @param  {std::string} id            : location id
 * @param  {std::vector<double>} square: four vertexes of the square area
 * @return {bool}                      : in square or not
 */
bool TrojanMap::inSquare(std::string id, std::vector<double> &square)
{
  return true;
}

/**
 * GetSubgraph: Give four vertexes of the square area, return a list of location
 * ids in the squares
 *
 * @param  {std::vector<double>} square         : four vertexes of the square
 * area
 * @return {std::vector<std::string>} subgraph  : list of location ids in the
 * square
 */
std::vector<std::string> TrojanMap::GetSubgraph(std::vector<double> &square)
{
  // include all the nodes in subgraph
  std::vector<std::string> subgraph;
  double left = square[0];
  double right = square[1];
  double upper = square[2];
  double lower = square[3];

  // loop through all nodes to check if they fall within bounds
  for (const auto &node : data)
  {
    double lat = node.second.lat;
    double lon = node.second.lon;

    if (lon >= left && lon <= right && lat <= upper && lat >= lower)
    {
      subgraph.push_back(node.first);
    }
  }

  return subgraph;
}

/**
 * Cycle Detection: Given four points of the square-shape subgraph, return true
 * if there is a cycle path inside the square, false otherwise.
 *
 * @param {std::vector<std::string>} subgraph: list of location ids in the
 * square
 * @param {std::vector<double>} square: four vertexes of the square area
 * @return {bool}: whether there is a cycle or not
 */
bool TrojanMap::CycleDetection(std::vector<std::string> &subgraph, std::vector<double> &square)
{
  std::unordered_map<std::string, bool> visited; // track visited nodes in subgraph

  // initialize all nodes in subgraph as unvisited
  for (const auto &node_id : subgraph)
  {
    visited[node_id] = false;
  }

  // stack for DFS traversal
  std::stack<std::pair<std::string, std::string>> stack;

  // perform DFS for cycle detection
  for (const auto &node_id : subgraph)
  {
    if (!visited[node_id]) // start DFS from unvisited node
    {
      stack.push({node_id, ""}); // push the start node and its parent (none for root)

      while (!stack.empty())
      {
        auto [current, parent] = stack.top();
        stack.pop();

        if (!visited[current]) // mark the node as visited
        {
          visited[current] = true;
        }

        // iterate through neighbors of the current node
        for (const auto &neighbor : data[current].neighbors)
        {
          // skip neighbors not in the subgraph
          if (visited.find(neighbor) == visited.end())
          {
            continue;
          }

          if (!visited[neighbor]) // unvisited neighbor
          {
            stack.push({neighbor, current}); // push neighbor with current as parent
          }
          else if (neighbor != parent) // back edge detected
          {
            return true; // cycle detected
          }
        }
      }
    }
  }
  return false; // no cycle found
}

/**
 * FindNearby: Given a class name C, a location name L and a number r,
 * find all locations in class C on the map near L with the range of r and
 * return a vector of string ids
 *
 * @param {std::string} className: the name of the class
 * @param {std::string} locationName: the name of the location
 * @param {double} r: search radius
 * @param {int} k: search numbers
 * @return {std::vector<std::string>}: location name that meets the requirements
 */
std::vector<std::string> TrojanMap::FindNearby(std::string attributesName, std::string name, double r, int k)
{
  std::vector<std::string> res;
  return res;
}

/**
 * Shortest Path to Visit All Nodes: Given a list of locations, return the shortest
 * path which visit all the places and no need to go back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::vector<std::string> }      : the shortest path
 */
std::vector<std::string> TrojanMap::TrojanPath(
    std::vector<std::string> &location_names)
{
  std::vector<std::string> res;
  return res;
}

/**
 * Given a vector of queries, find whether there is a path between the two locations with the constraint of the gas tank.
 *
 * @param  {std::vector<std::pair<double, std::vector<std::string>>>} Q : a list of queries
 * @return {std::vector<bool> }      : existence of the path
 */
std::vector<bool> TrojanMap::Queries(const std::vector<std::pair<double, std::vector<std::string>>> &q)
{
  std::vector<bool> ans(q.size());
  return ans;
}

/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 * We have provided the code for you. Please do not need to change this function.
 */
void TrojanMap::CreateGraphFromCSVFile()
{
  // Do not change this function
  std::fstream fin;
  fin.open("src/lib/data.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line))
  {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ','))
    {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '{'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '}'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else
      {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        if (isalpha(word[0]))
          n.attributes.insert(word);
        if (isdigit(word[0]))
          n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
}
