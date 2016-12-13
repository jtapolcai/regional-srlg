/* -*- mode: C++; indent-tabs-mode: nil; -*-
 *
 * This file is a part of LEMON, a generic C++ optimization library.
 *
 * Copyright (C) 2003-2010
 * Egervary Jeno Kombinatorikus Optimalizalasi Kutatocsoport
 * (Egervary Research Group on Combinatorial Optimization, EGRES).
 *
 * Permission to use, modify and distribute this software is granted
 * provided that this copyright notice appears in all copies. For
 * precise terms see the accompanying LICENSE file.
 *
 * This software is provided "AS IS" with no warranty of any kind,
 * express or implied, and with no claim as to its suitability for any
 * purpose.
 *
 */

#ifndef LEMON_GROSSO_LOCATELLI_PULLAN_MC_H
#define LEMON_GROSSO_LOCATELLI_PULLAN_MC_H

/// \ingroup approx_algs
///
/// \file
/// \brief The iterated local search algorithm of Grosso, Locatelli, and Pullan
/// for the maximum clique problem

#include <vector>
#include <limits>
#include <lemon/core.h>
#include <lemon/random.h>

namespace lemon {

  /// \addtogroup approx_algs
  /// @{

  /// \brief Implementation of the iterated local search algorithm of Grosso,
  /// Locatelli, and Pullan for the maximum clique problem
  ///
  /// \ref GrossoLocatelliPullanMc implements the iterated local search
  /// algorithm of Grosso, Locatelli, and Pullan for solving the \e maximum
  /// \e clique \e problem \ref grosso08maxclique.
  /// It is to find the largest complete subgraph (\e clique) in an
  /// undirected graph, i.e., the largest set of nodes where each
  /// pair of nodes is connected.
  ///
  /// This class provides a simple but highly efficient and robust heuristic
  /// method that quickly finds a large clique, but not necessarily the
  /// largest one.
  ///
  /// \tparam GR The undirected graph type the algorithm runs on.
  ///
  /// \note %GrossoLocatelliPullanMc provides three different node selection
  /// rules, from which the most powerful one is used by default.
  /// For more information, see \ref SelectionRule.
  template <typename GR>
  class GrossoLocatelliPullanMc
  {
  public:

    /// \brief Constants for specifying the node selection rule.
    ///
    /// Enum type containing constants for specifying the node selection rule
    /// for the \ref run() function.
    ///
    /// During the algorithm, nodes are selected for addition to the current
    /// clique according to the applied rule.
    /// In general, the PENALTY_BASED rule turned out to be the most powerful
    /// and the most robust, thus it is the default option.
    /// However, another selection rule can be specified using the \ref run()
    /// function with the proper parameter.
    enum SelectionRule {

      /// A node is selected randomly without any evaluation at each step.
      RANDOM,

      /// A node of maximum degree is selected randomly at each step.
      DEGREE_BASED,

      /// A node of minimum penalty is selected randomly at each step.
      /// The node penalties are updated adaptively after each stage of the
      /// search process.
      PENALTY_BASED
    };

  private:

    TEMPLATE_GRAPH_TYPEDEFS(GR);

    typedef std::vector<int> IntVector;
    typedef std::vector<char> BoolVector;
    typedef std::vector<BoolVector> BoolMatrix;
    // Note: vector<char> is used instead of vector<bool> for efficiency reasons

    const GR &_graph;
    IntNodeMap _id;

    // Internal matrix representation of the graph
    BoolMatrix _gr;
    int _n;

    // The current clique
    BoolVector _clique;
    int _size;

    // The best clique found so far
    BoolVector _best_clique;
    int _best_size;

    // The "distances" of the nodes from the current clique.
    // _delta[u] is the number of nodes in the clique that are
    // not connected with u.
    IntVector _delta;

    // The current tabu set
    BoolVector _tabu;

    // Random number generator
    Random _rnd;

  private:

    // Implementation of the RANDOM node selection rule.
    class RandomSelectionRule
    {
    private:

      // References to the algorithm instance
      const BoolVector &_clique;
      const IntVector  &_delta;
      const BoolVector &_tabu;
      Random &_rnd;

      // Pivot rule data
      int _n;

    public:

      // Constructor
      RandomSelectionRule(GrossoLocatelliPullanMc &mc) :
        _clique(mc._clique), _delta(mc._delta), _tabu(mc._tabu),
        _rnd(mc._rnd), _n(mc._n)
      {}

      // Return a node index for a feasible add move or -1 if no one exists
      int nextFeasibleAddNode() const {
        int start_node = _rnd[_n];
        for (int i = start_node; i != _n; i++) {
          if (_delta[i] == 0 && !_tabu[i]) return i;
        }
        for (int i = 0; i != start_node; i++) {
          if (_delta[i] == 0 && !_tabu[i]) return i;
        }
        return -1;
      }

      // Return a node index for a feasible swap move or -1 if no one exists
      int nextFeasibleSwapNode() const {
        int start_node = _rnd[_n];
        for (int i = start_node; i != _n; i++) {
          if (!_clique[i] && _delta[i] == 1 && !_tabu[i]) return i;
        }
        for (int i = 0; i != start_node; i++) {
          if (!_clique[i] && _delta[i] == 1 && !_tabu[i]) return i;
        }
        return -1;
      }

      // Return a node index for an add move or -1 if no one exists
      int nextAddNode() const {
        int start_node = _rnd[_n];
        for (int i = start_node; i != _n; i++) {
          if (_delta[i] == 0) return i;
        }
        for (int i = 0; i != start_node; i++) {
          if (_delta[i] == 0) return i;
        }
        return -1;
      }

      // Update internal data structures between stages (if necessary)
      void update() {}

    }; //class RandomSelectionRule


    // Implementation of the DEGREE_BASED node selection rule.
    class DegreeBasedSelectionRule
    {
    private:

      // References to the algorithm instance
      const BoolVector &_clique;
      const IntVector  &_delta;
      const BoolVector &_tabu;
      Random &_rnd;

      // Pivot rule data
      int _n;
      IntVector _deg;

    public:

      // Constructor
      DegreeBasedSelectionRule(GrossoLocatelliPullanMc &mc) :
        _clique(mc._clique), _delta(mc._delta), _tabu(mc._tabu),
        _rnd(mc._rnd), _n(mc._n), _deg(_n)
      {
        for (int i = 0; i != _n; i++) {
          int d = 0;
          BoolVector &row = mc._gr[i];
          for (int j = 0; j != _n; j++) {
            if (row[j]) d++;
          }
          _deg[i] = d;
        }
      }

      // Return a node index for a feasible add move or -1 if no one exists
      int nextFeasibleAddNode() const {
        int start_node = _rnd[_n];
        int node = -1, max_deg = -1;
        for (int i = start_node; i != _n; i++) {
          if (_delta[i] == 0 && !_tabu[i] && _deg[i] > max_deg) {
            node = i;
            max_deg = _deg[i];
          }
        }
        for (int i = 0; i != start_node; i++) {
          if (_delta[i] == 0 && !_tabu[i] && _deg[i] > max_deg) {
            node = i;
            max_deg = _deg[i];
          }
        }
        return node;
      }

      // Return a node index for a feasible swap move or -1 if no one exists
      int nextFeasibleSwapNode() const {
        int start_node = _rnd[_n];
        int node = -1, max_deg = -1;
        for (int i = start_node; i != _n; i++) {
          if (!_clique[i] && _delta[i] == 1 && !_tabu[i] &&
              _deg[i] > max_deg) {
            node = i;
            max_deg = _deg[i];
          }
        }
        for (int i = 0; i != start_node; i++) {
          if (!_clique[i] && _delta[i] == 1 && !_tabu[i] &&
              _deg[i] > max_deg) {
            node = i;
            max_deg = _deg[i];
          }
        }
        return node;
      }

      // Return a node index for an add move or -1 if no one exists
      int nextAddNode() const {
        int start_node = _rnd[_n];
        int node = -1, max_deg = -1;
        for (int i = start_node; i != _n; i++) {
          if (_delta[i] == 0 && _deg[i] > max_deg) {
            node = i;
            max_deg = _deg[i];
          }
        }
        for (int i = 0; i != start_node; i++) {
          if (_delta[i] == 0 && _deg[i] > max_deg) {
            node = i;
            max_deg = _deg[i];
          }
        }
        return node;
      }

      // Update internal data structures between stages (if necessary)
      void update() {}

    }; //class DegreeBasedSelectionRule


    // Implementation of the PENALTY_BASED node selection rule.
    class PenaltyBasedSelectionRule
    {
    private:

      // References to the algorithm instance
      const BoolVector &_clique;
      const IntVector  &_delta;
      const BoolVector &_tabu;
      Random &_rnd;

      // Pivot rule data
      int _n;
      IntVector _penalty;

    public:

      // Constructor
      PenaltyBasedSelectionRule(GrossoLocatelliPullanMc &mc) :
        _clique(mc._clique), _delta(mc._delta), _tabu(mc._tabu),
        _rnd(mc._rnd), _n(mc._n), _penalty(_n, 0)
      {}

      // Return a node index for a feasible add move or -1 if no one exists
      int nextFeasibleAddNode() const {
        int start_node = _rnd[_n];
        int node = -1, min_p = std::numeric_limits<int>::max();
        for (int i = start_node; i != _n; i++) {
          if (_delta[i] == 0 && !_tabu[i] && _penalty[i] < min_p) {
            node = i;
            min_p = _penalty[i];
          }
        }
        for (int i = 0; i != start_node; i++) {
          if (_delta[i] == 0 && !_tabu[i] && _penalty[i] < min_p) {
            node = i;
            min_p = _penalty[i];
          }
        }
        return node;
      }

      // Return a node index for a feasible swap move or -1 if no one exists
      int nextFeasibleSwapNode() const {
        int start_node = _rnd[_n];
        int node = -1, min_p = std::numeric_limits<int>::max();
        for (int i = start_node; i != _n; i++) {
          if (!_clique[i] && _delta[i] == 1 && !_tabu[i] &&
              _penalty[i] < min_p) {
            node = i;
            min_p = _penalty[i];
          }
        }
        for (int i = 0; i != start_node; i++) {
          if (!_clique[i] && _delta[i] == 1 && !_tabu[i] &&
              _penalty[i] < min_p) {
            node = i;
            min_p = _penalty[i];
          }
        }
        return node;
      }

      // Return a node index for an add move or -1 if no one exists
      int nextAddNode() const {
        int start_node = _rnd[_n];
        int node = -1, min_p = std::numeric_limits<int>::max();
        for (int i = start_node; i != _n; i++) {
          if (_delta[i] == 0 && _penalty[i] < min_p) {
            node = i;
            min_p = _penalty[i];
          }
        }
        for (int i = 0; i != start_node; i++) {
          if (_delta[i] == 0 && _penalty[i] < min_p) {
            node = i;
            min_p = _penalty[i];
          }
        }
        return node;
      }

      // Update internal data structures between stages (if necessary)
      void update() {}

    }; //class PenaltyBasedSelectionRule

  public:

    /// \brief Constructor.
    ///
    /// Constructor.
    /// The global \ref rnd "random number generator instance" is used
    /// during the algorithm.
    ///
    /// \param graph The undirected graph the algorithm runs on.
    GrossoLocatelliPullanMc(const GR& graph) :
      _graph(graph), _id(_graph), _rnd(rnd)
    {}

    /// \brief Constructor with random seed.
    ///
    /// Constructor with random seed.
    ///
    /// \param graph The undirected graph the algorithm runs on.
    /// \param seed Seed value for the internal random number generator
    /// that is used during the algorithm.
    GrossoLocatelliPullanMc(const GR& graph, int seed) :
      _graph(graph), _id(_graph), _rnd(seed)
    {}

    /// \brief Constructor with random number generator.
    ///
    /// Constructor with random number generator.
    ///
    /// \param graph The undirected graph the algorithm runs on.
    /// \param random A random number generator that is used during the
    /// algorithm.
    GrossoLocatelliPullanMc(const GR& graph, const Random& random) :
      _graph(graph), _id(_graph), _rnd(random)
    {}

    /// \name Execution Control
    /// @{

    /// \brief Runs the algorithm.
    ///
    /// This function runs the algorithm.
    ///
    /// \param step_num The maximum number of node selections (steps)
    /// during the search process.
    /// This parameter controls the running time and the success of the
    /// algorithm. For larger values, the algorithm runs slower but it more
    /// likely finds larger cliques. For smaller values, the algorithm is
    /// faster but probably gives worse results.
    /// \param rule The node selection rule. For more information, see
    /// \ref SelectionRule.
    ///
    /// \return The size of the found clique.
    int run(int step_num = 100000,
            SelectionRule rule = PENALTY_BASED)
    {
      init();
      switch (rule) {
        case RANDOM:
          return start<RandomSelectionRule>(step_num);
        case DEGREE_BASED:
          return start<DegreeBasedSelectionRule>(step_num);
        case PENALTY_BASED:
          return start<PenaltyBasedSelectionRule>(step_num);
      }
      return 0; // avoid warning
    }

    /// @}

    /// \name Query Functions
    /// @{

    /// \brief The size of the found clique
    ///
    /// This function returns the size of the found clique.
    ///
    /// \pre run() must be called before using this function.
    int cliqueSize() const {
      return _best_size;
    }

    /// \brief Gives back the found clique in a \c bool node map
    ///
    /// This function gives back the characteristic vector of the found
    /// clique in the given node map.
    /// It must be a \ref concepts::WriteMap "writable" node map with
    /// \c bool (or convertible) value type.
    ///
    /// \pre run() must be called before using this function.
    template <typename CliqueMap>
    void cliqueMap(CliqueMap &map) const {
      for (NodeIt n(_graph); n != INVALID; ++n) {
        map[n] = static_cast<bool>(_best_clique[_id[n]]);
      }
    }

    /// \brief Iterator to list the nodes of the found clique
    ///
    /// This iterator class lists the nodes of the found clique.
    /// Before using it, you must allocate a GrossoLocatelliPullanMc instance
    /// and call its \ref GrossoLocatelliPullanMc::run() "run()" method.
    ///
    /// The following example prints out the IDs of the nodes in the found
    /// clique.
    /// \code
    ///   GrossoLocatelliPullanMc<Graph> mc(g);
    ///   mc.run();
    ///   for (GrossoLocatelliPullanMc<Graph>::CliqueNodeIt n(mc);
    ///        n != INVALID; ++n)
    ///   {
    ///     std::cout << g.id(n) << std::endl;
    ///   }
    /// \endcode
    class CliqueNodeIt
    {
    private:
      NodeIt _it;
      BoolNodeMap _map;

    public:

      /// Constructor

      /// Constructor.
      /// \param mc The algorithm instance.
      CliqueNodeIt(const GrossoLocatelliPullanMc &mc)
       : _map(mc._graph)
      {
        mc.cliqueMap(_map);
        for (_it = NodeIt(mc._graph); _it != INVALID && !_map[_it]; ++_it) ;
      }

      /// Conversion to \c Node
      operator Node() const { return _it; }

      bool operator==(Invalid) const { return _it == INVALID; }
      bool operator!=(Invalid) const { return _it != INVALID; }

      /// Next node
      CliqueNodeIt &operator++() {
        for (++_it; _it != INVALID && !_map[_it]; ++_it) ;
        return *this;
      }

      /// Postfix incrementation

      /// Postfix incrementation.
      ///
      /// \warning This incrementation returns a \c Node, not a
      /// \c CliqueNodeIt as one may expect.
      typename GR::Node operator++(int) {
        Node n=*this;
        ++(*this);
        return n;
      }

    };

    /// @}

  private:

    // Adds a node to the current clique
    void addCliqueNode(int u) {
      if (_clique[u]) return;
      _clique[u] = true;
      _size++;
      BoolVector &row = _gr[u];
      for (int i = 0; i != _n; i++) {
        if (!row[i]) _delta[i]++;
      }
    }

    // Removes a node from the current clique
    void delCliqueNode(int u) {
      if (!_clique[u]) return;
      _clique[u] = false;
      _size--;
      BoolVector &row = _gr[u];
      for (int i = 0; i != _n; i++) {
        if (!row[i]) _delta[i]--;
      }
    }

    // Initialize data structures
    void init() {
      _n = countNodes(_graph);
      int ui = 0;
      for (NodeIt u(_graph); u != INVALID; ++u) {
        _id[u] = ui++;
      }
      _gr.clear();
      _gr.resize(_n, BoolVector(_n, false));
      ui = 0;
      for (NodeIt u(_graph); u != INVALID; ++u) {
        for (IncEdgeIt e(_graph, u); e != INVALID; ++e) {
          int vi = _id[_graph.runningNode(e)];
          _gr[ui][vi] = true;
          _gr[vi][ui] = true;
        }
        ++ui;
      }

      _clique.clear();
      _clique.resize(_n, false);
      _size = 0;
      _best_clique.clear();
      _best_clique.resize(_n, false);
      _best_size = 0;
      _delta.clear();
      _delta.resize(_n, 0);
      _tabu.clear();
      _tabu.resize(_n, false);
    }

    // Executes the algorithm
    template <typename SelectionRuleImpl>
    int start(int max_select) {
      // Options for the restart rule
      const bool delta_based_restart = true;
      const int restart_delta_limit = 4;

      if (_n == 0) return 0;
      if (_n == 1) {
        _best_clique[0] = true;
        _best_size = 1;
        return _best_size;
      }

      // Iterated local search
      SelectionRuleImpl sel_method(*this);
      int select = 0;
      IntVector restart_nodes;

      while (select < max_select) {

        // Perturbation/restart
        if (delta_based_restart) {
          restart_nodes.clear();
          for (int i = 0; i != _n; i++) {
            if (_delta[i] >= restart_delta_limit)
              restart_nodes.push_back(i);
          }
        }
        int rs_node = -1;
        if (restart_nodes.size() > 0) {
          rs_node = restart_nodes[_rnd[restart_nodes.size()]];
        } else {
          rs_node = _rnd[_n];
        }
        BoolVector &row = _gr[rs_node];
        for (int i = 0; i != _n; i++) {
          if (_clique[i] && !row[i]) delCliqueNode(i);
        }
        addCliqueNode(rs_node);

        // Local search
        _tabu.clear();
        _tabu.resize(_n, false);
        bool tabu_empty = true;
        int max_swap = _size;
        while (select < max_select) {
          select++;
          int u;
          if ((u = sel_method.nextFeasibleAddNode()) != -1) {
            // Feasible add move
            addCliqueNode(u);
            if (tabu_empty) max_swap = _size;
          }
          else if ((u = sel_method.nextFeasibleSwapNode()) != -1) {
            // Feasible swap move
            int v = -1;
            BoolVector &row = _gr[u];
            for (int i = 0; i != _n; i++) {
              if (_clique[i] && !row[i]) {
                v = i;
                break;
              }
            }
            addCliqueNode(u);
            delCliqueNode(v);
            _tabu[v] = true;
            tabu_empty = false;
            if (--max_swap <= 0) break;
          }
          else if ((u = sel_method.nextAddNode()) != -1) {
            // Non-feasible add move
            addCliqueNode(u);
          }
          else break;
        }
        if (_size > _best_size) {
          _best_clique = _clique;
          _best_size = _size;
          if (_best_size == _n) return _best_size;
        }
        sel_method.update();
      }

      return _best_size;
    }

  }; //class GrossoLocatelliPullanMc

  ///@}

} //namespace lemon

#endif //LEMON_GROSSO_LOCATELLI_PULLAN_MC_H
