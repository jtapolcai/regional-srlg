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

#ifndef LEMON_GUROBI_H
#define LEMON_GUROBI_H

///\file
///\brief Header of the LEMON-GLPK lp solver interface.
///\ingroup lp_group

#include <lemon/lp_base.h>

extern "C"{
  #include <gurobi_c.h>
};
//struct GRBenv;
//struct GRBmodel;

namespace lemon {

  /// \brief Base interface for the GLPK LP and MIP solver
  ///
  /// This class implements the common interface of the GLPK LP and MIP solver.
  /// \ingroup lp_group
  class GurobiBase : virtual public LpBase {
  protected:

    GRBenv *env;
    GRBmodel *model;

    std::map<int,std::pair<Value,Value> > slacks;   ///< Variable to store the bound for the ranged constraints.

    GurobiBase();
    GurobiBase(const GurobiBase&);
    virtual ~GurobiBase();

  protected:

    virtual int _addCol();
    virtual int _addRow();
    virtual int _addRow(Value l, ExprIterator b, ExprIterator e, Value u);

    virtual void _eraseCol(int i);
    virtual void _eraseRow(int i);

    virtual void _eraseColId(int i);
    virtual void _eraseRowId(int i);

    virtual void _getColName(int col, std::string& name) const;
    virtual void _setColName(int col, const std::string& name);
    virtual int _colByName(const std::string& name) const;

    virtual void _getRowName(int row, std::string& name) const;
    virtual void _setRowName(int row, const std::string& name);
    virtual int _rowByName(const std::string& name) const;

    virtual void _setRowCoeffs(int i, ExprIterator b, ExprIterator e);
    virtual void _getRowCoeffs(int i, InsertIterator b) const;

    virtual void _setColCoeffs(int i, ExprIterator b, ExprIterator e);
    virtual void _getColCoeffs(int i, InsertIterator b) const;

    virtual void _setCoeff(int row, int col, Value value);
    virtual Value _getCoeff(int row, int col) const;

    virtual void _setColLowerBound(int i, Value value);
    virtual Value _getColLowerBound(int i) const;

    virtual void _setColUpperBound(int i, Value value);
    virtual Value _getColUpperBound(int i) const;

    virtual void _setRowLowerBound(int i, Value value);
    virtual Value _getRowLowerBound(int i) const;

    virtual void _setRowUpperBound(int i, Value value);
    virtual Value _getRowUpperBound(int i) const;

    virtual void _setObjCoeffs(ExprIterator b, ExprIterator e);
    virtual void _getObjCoeffs(InsertIterator b) const;

    virtual void _setObjCoeff(int i, Value obj_coef);
    virtual Value _getObjCoeff(int i) const;

    virtual void _setSense(Sense);
    virtual Sense _getSense() const;

    virtual void _clear();

    virtual void _messageLevel(MessageLevel level);

  protected:

    int _message_level;

  public:

    GRBenv* gurobiEnv(){ return env; }
    GRBmodel* gurobiModel(){ return model; }

    const GRBenv* gurobiEnv()const{ return env; }
    const GRBmodel* gurobiModel()const{ return model; }

    ///Returns the constraint identifier understood by GLPK.
    int lpxRow(Row r) const { return rows(id(r)); }

    ///Returns the variable identifier understood by GLPK.
    int lpxCol(Col c) const { return cols(id(c)); }

  };

  /// \brief Interface for the GLPK LP solver
  ///
  /// This class implements an interface for the GLPK LP solver.
  ///\ingroup lp_group
  class GurobiLp : public LpSolver, public GurobiBase {
  public:

    ///\e
    GurobiLp();
    ///\e
    GurobiLp(const GurobiLp&);

    ///\e
    virtual GurobiLp* cloneSolver() const;
    ///\e
    virtual GurobiLp* newSolver() const;

  private:

    mutable std::vector<double> _primal_ray;
    mutable std::vector<double> _dual_ray;

    void _clear_temporals();
    SolveExitStatus convertStatus(int status);

  protected:

    virtual const char* _solverName() const;

    virtual SolveExitStatus _solve();
    virtual Value _getPrimal(int i) const;
    virtual Value _getDual(int i) const;

    virtual Value _getPrimalValue() const;

    virtual VarStatus _getColStatus(int i) const;
    virtual VarStatus _getRowStatus(int i) const;

    virtual Value _getPrimalRay(int i) const;
    virtual Value _getDualRay(int i) const;

    virtual ProblemType _getPrimalType() const;
    virtual ProblemType _getDualType() const;

  public:

  private:

    bool _presolve;

  public:

    ///Turns on or off the presolver

    ///Turns on (\c b is \c true) or off (\c b is \c false) the presolver
    ///
    ///The presolver is off by default.
    void presolver(bool presolve);

    SolveExitStatus solvePrimal();
    SolveExitStatus solveDual();

  };

  /// \brief Interface for the GLPK MIP solver
  ///
  /// This class implements an interface for the GLPK MIP solver.
  ///\ingroup lp_group
  class GurobiMip : public MipSolver, public GurobiBase {
  public:

    ///\e
    GurobiMip();
    ///\e
    GurobiMip(const GurobiMip&);

    virtual GurobiMip* cloneSolver() const;
    virtual GurobiMip* newSolver() const;

  protected:

    virtual const char* _solverName() const;

    virtual ColTypes _getColType(int col) const;
    virtual void _setColType(int col, ColTypes col_type);

    virtual SolveExitStatus _solve();
    virtual ProblemType _getType() const;
    virtual Value _getSol(int i) const;
    virtual Value _getSolValue() const;

  private:

    SolveExitStatus convertStatus(int status);

  };


} //END OF NAMESPACE LEMON

#endif //LEMON_GUROBI_H

