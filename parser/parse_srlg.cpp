/**
 \file parse_srlg.cpp
 \brief A demo fucntion to parse the SRLGs


Usage:
  ./parse_srlg [lgf file]

Where:

[lgf file]
     The undirected input topology with srlgs
 
 
 \date    2016 december
 \author  Janos Tapolcai
 \author  tapolcai@tmit.bme.hu
 ***************************************************************/

#include <lemon/smart_graph.h>
#include <lemon/lgf_reader.h>
#include <lemon/dim2.h>

#include <queue>

#include "make_names.h"

using namespace lemon;
using namespace std;

typedef SmartGraph Graph;
GRAPH_TYPEDEFS(Graph);
typedef dim2::Point<double> Point;
typedef Graph::NodeMap<Point> CoordMap;


struct SRLGSection {
    queue<vector<int> >& _data;
    SRLGSection(queue<vector<int> >& data) : _data(data) {}
    void operator()(const std::string& line) {
        std::istringstream ls(line);
        int value;
        vector<int> srlg_edges;
        while (ls >> value) srlg_edges.push_back(value);
        _data.push(srlg_edges);
    }
};

int main(int argc,char **argv)
{
	if (argc<=1) {
		cerr<<"Usage: "<<argv[0]<<" [lgf file] \n";
		return -1;
	}

	string lgf_file_name=argv[1];
	ifstream lgf_file(lgf_file_name.c_str());
	if (!lgf_file) {
		cerr<<"Error opening lgf file of "<<lgf_file_name;
		return -1;
	}
    Graph g;
	GraphReader<Graph> reader(g,lgf_file);
	try {
		reader.run();
	} catch (FormatError& error) {
		cerr<<error.what() << endl;
		return -1;
	}

    ifstream srg_file(lgf_file_name.c_str());
    queue<vector<int> > edges_q;
    SectionReader srg_reader(srg_file);
    try {
        srg_reader.sectionLines("srlgs", SRLGSection(edges_q));
        srg_reader.run();
    } catch (FormatError& error) {
        cerr<<error.what() << endl;
        return -1;
    }
    cout << "\nNodes:\n";
    for(NodeIt n(g);n!=INVALID; ++n) cout << node_name(n,g)<<" ";
    cout << "\nEdges:\n";
    for(EdgeIt e(g);e!=INVALID; ++e) cout << edge_name(e,g)<<" ";
    int count=0;
    for(int i = edges_q.size(); i > 0; i--){
        int edge_num = 0;
        cout << "\nSRG"<<count<<":";
        for(vector<int>::iterator it=edges_q.front().begin(); it!=edges_q.front().end(); it++){
            Edge e=g.edgeFromId(*it);
            edge_num++;
            cout << " " << edge_name(e, g);
        }
        edges_q.pop();
        count++;
    }
    return 0;
}

