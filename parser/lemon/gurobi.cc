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

///\file
///\brief Implementation of the LEMON GUROBI LP and MIP solver interface.

#include <lemon/gurobi.h>

extern "C"{
  #include <gurobi_c.h>
};

#include <lemon/assert.h>

namespace lemon{

  // GurobiBase members

    GurobiBase::GurobiBase():LpBase(),env(0),model(0){

        int error=GRBloadenv(&env,0);
        if(error || !env){
            // error
        }

        error=GRBsetintparam(env,"InfUnbdInfo",1);
        error=GRBsetintparam(env,"OutputFlag",0); // turn off logging


        error=GRBnewmodel(env,&model,"lemon_lp",0,0,0,0,0,0);
        if(error){
            // error
        }

        messageLevel(MESSAGE_NOTHING);
    }


    GurobiBase::GurobiBase(const GurobiBase &other):LpBase(),env(0){

        int error=GRBloadenv(&env,0);

        if(error || !env){
            // error
        }
        error=GRBsetintparam(env,"InfUnbdInfo",1);
        error=GRBsetintparam(env,"OutputFlag",0); // turn off logging

        model=GRBcopymodel(other.model);

        rows=other.rows;
        cols=other.cols;

        messageLevel(MESSAGE_NOTHING);
    }

    GurobiBase::~GurobiBase(){
        GRBfreemodel(model);
        GRBfreeenv(env);
    }

    int GurobiBase::_addCol(){
        int i;
        int error=GRBgetintattr(model,"NumVars",&i);
        error=GRBaddvar(model,0,0,0,0,-GRB_INFINITY,GRB_INFINITY,GRB_CONTINUOUS,0);
        GRBupdatemodel(model);
        return i;
    }

    int GurobiBase::_addRow(){
        int i;
        int error=GRBgetintattr(model,"NumConstrs",&i);
        error=GRBaddconstr(model,0,0,0,GRB_EQUAL,0.0,0);
        GRBupdatemodel(model);
        return i;
    }

    int GurobiBase::_addRow(Value lo,ExprIterator b,ExprIterator e,Value up){

        int error=0;

        int i;
        error=GRBgetintattr(model,"NumConstrs",&i);

        std::vector<int> indexes;
        std::vector<Value> values;

        for(ExprIterator it=b;it!=e;++it){
            if(it->second!=0){
                indexes.push_back(it->first);
                values.push_back(it->second);
            }
        }

        if(lo==-INF){
            error=GRBaddconstr(model,values.size(),&indexes.front(),&values.front(),GRB_LESS_EQUAL,static_cast<double>(up),0);
        }
        else if(up==INF){
            error=GRBaddconstr(model,values.size(),&indexes.front(),&values.front(),GRB_GREATER_EQUAL,static_cast<double>(lo),0);
        }
        else if(lo==up){
            error=GRBaddconstr(model,values.size(),&indexes.front(),&values.front(),GRB_EQUAL,static_cast<double>(up),0);
        }
        else{
            slacks[i]=std::make_pair(lo,up);
            error=GRBaddrangeconstr(model,values.size(),&indexes.front(),&values.front(),static_cast<double>(lo),static_cast<double>(up),0);
        }

        GRBupdatemodel(model);
        return i;
    }

    void GurobiBase::_eraseCol(int i){
        int error=GRBdelvars(model,1,&i);
        GRBupdatemodel(model);
    }

    void GurobiBase::_eraseRow(int i) {
        if(slacks.find(i)!=slacks.end()) slacks.erase(i);
        int error=GRBdelconstrs(model,1,&i);
        GRBupdatemodel(model);
    }

    void GurobiBase::_eraseColId(int i) {
        cols.eraseIndex(i);
        cols.shiftIndices(i);
    }

    void GurobiBase::_eraseRowId(int i) {
        rows.eraseIndex(i);
        rows.shiftIndices(i);
    }

    void GurobiBase::_getColName(int c,std::string &name)const{
        char *str;
        int error=GRBgetstrattrelement(model,"VarName",c,&str);
        if(str) name = str;
        else name.clear();
    }

    void GurobiBase::_setColName(int c,const std::string &name){
        int error=GRBsetstrattrelement(model,"VarName",c,const_cast<char*>(name.c_str()));
        GRBupdatemodel(model);
    }

    int GurobiBase::_colByName(const std::string& name)const{
        int length;
        int error=GRBgetintattr(model,"NumVars",&length);
        for(int i=0;i<length;i++){
            std::string name1;
            _getColName(i,name1);
            if(name==name1) return i;
        }
        return -1;
  }

    void GurobiBase::_getRowName(int r, std::string& name)const{
        char *str;
        int error=GRBgetstrattrelement(model,"ConstrName",r,&str);
        if(str) name = str;
        else name.clear();
    }

    void GurobiBase::_setRowName(int r,const std::string &name){
        int error=GRBsetstrattrelement(model,"ConstrName",r,const_cast<char*>(name.c_str()));
        GRBupdatemodel(model);
    }


    int GurobiBase::_rowByName(const std::string& name) const {
        int length;
        int error=GRBgetintattr(model,"NumConstrs",&length);
        for(int i=0;i<length;i++){
            std::string name1;
            _getRowName(i,name1);
            if(name==name1) return i;
        }
        return -1;
    }


    void GurobiBase::_setRowCoeffs(int i,ExprIterator b,ExprIterator e){
        std::vector<int> rows;
        std::vector<int> indexes;
        std::vector<double> values;

        for(ExprIterator it=b;it!=e;++it){
            if(it->second!=0){
                rows.push_back(i);
                indexes.push_back(it->first);
                values.push_back(static_cast<double>(it->second));
            }
        }

        int error=GRBchgcoeffs(model,values.size(),&rows.front(),&indexes.front(),&values.front());
        GRBupdatemodel(model);
    }

    void GurobiBase::_getRowCoeffs(int ix,InsertIterator b)const{
        int length;
        int error=GRBgetintattr(model,"NumVars",&length);

        for(int i=1;i<=length;++i){
            double a;
            error=GRBgetcoeff(model,ix,i,&a);
            *b=std::make_pair(i,static_cast<Value>(a));
            ++b;
        }
    }

    void GurobiBase::_setColCoeffs(int ix,ExprIterator b,ExprIterator e){

        std::vector<int> cols;
        std::vector<int> indexes;
        std::vector<double> values;

        for(ExprIterator it=b;it!=e;++it){
            if(it->second!=0){
                cols.push_back(ix);
                indexes.push_back(it->first);
                values.push_back(static_cast<double>(it->second));
            }
        }

        int error=GRBchgcoeffs(model,values.size(),&indexes.front(),&cols.front(),&values.front());
        GRBupdatemodel(model);
    }

    void GurobiBase::_getColCoeffs(int ix, InsertIterator b) const {
        int length;
        int error=GRBgetintattr(model,"NumConstrs",&length);

        for(int i=1;i<=length;++i){
            double a;
            error=GRBgetcoeff(model,i,ix,&a);
            *b=std::make_pair(i,static_cast<Value>(a));
            ++b;
        }
    }

    void GurobiBase::_setCoeff(int ix,int jx,Value value){
        double va=static_cast<double>(value);
        int error=GRBchgcoeffs(model,1,&ix,&jx,&va);
        GRBupdatemodel(model);
    }

    GurobiBase::Value GurobiBase::_getCoeff(int ix, int jx) const {
        double a;
        int error=GRBgetcoeff(model,ix,jx,&a);
        return static_cast<Value>(a);
    }

    void GurobiBase::_setColLowerBound(int i,Value lo){
        LEMON_ASSERT(lo!=INF,"Invalid bound");
        double lb=static_cast<double>(lo);
        int error=GRBsetdblattrelement(model,"LB",i,lb);
        GRBupdatemodel(model);
    }

    GurobiBase::Value GurobiBase::_getColLowerBound(int i)const{
        double lb;
        int error=GRBgetdblattrelement(model,"LB",i,&lb);
        if(lb!=-GRB_INFINITY) return static_cast<Value>(lb);
        else return -INF;
    }

    void GurobiBase::_setColUpperBound(int i,Value up){
        LEMON_ASSERT(up!=-INF,"Invalid bound");
        double ub=static_cast<double>(up);
        int error=GRBsetdblattrelement(model,"UB",i,ub);
        GRBupdatemodel(model);
    }

    GurobiBase::Value GurobiBase::_getColUpperBound(int i)const{
        double ub;
        int error=GRBgetdblattrelement(model,"UB",i,&ub);
        if(ub!=GRB_INFINITY) return static_cast<Value>(ub);
        else return INF;
    }

    /*************************************/

    void GurobiBase::_setRowLowerBound(int i,Value lo) {
        LEMON_ASSERT(lo!=INF,"Invalid bound");

        char sense;
        int error=0;
        int ra[2];
        ra[1]=i;

        error=GRBgetcharattrelement(model,"Sense",i,&sense);

        if(sense=='>'){
            error=GRBsetdblattrelement(model,"RHS",i,lo);
        }
        else if(sense=='='){
            if(slacks.find(i)==slacks.end()){
                error=GRBsetcharattrelement(model,"Sense",i,'>');
                error=GRBsetdblattrelement(model,"RHS",i,lo);
            }
            else{

                std::vector<int> indexes;
                std::vector<double> values;
                int length;
                error=GRBgetintattr(model,"NumVars",&length);
                for(int ic=1;ic<=length;++ic){
                    double a;
                    error=GRBgetcoeff(model,i,ic,&a);
                    if(a!=0){
                        indexes.push_back(ic);
                        values.push_back(a);
                    }
                }
                std::string name;
                _getRowName(i,name);

                double up;
                error=GRBgetdblattrelement(model,"RHS",i,&up);

                error=GRBdelconstrs(model,1,ra);
                GRBupdatemodel(model);
                slacks[i]=std::make_pair(lo,up);
                error=GRBaddrangeconstr(model,values.size(),&indexes.front(),&values.front(),static_cast<double>(lo),static_cast<double>(up),const_cast<char*>(name.c_str()));

            }
        }
        else{
            Value up;
            if(slacks.find(i)!=slacks.end()){
                up=slacks[i].second;
            }
            else{
                error=GRBsetdblattrelement(model,"RHS",i,up);
            }

            if(up==lo && slacks.find(i)==slacks.end()){
                    error=GRBsetcharattrelement(model,"Sense",i,'=');
                    error=GRBsetdblattrelement(model,"RHS",i,lo);
            }
            else{
                std::vector<int> indexes;
                std::vector<double> values;
                int length;
                error=GRBgetintattr(model,"NumVars",&length);
                for(int ic=1;ic<=length;++ic){
                    double a;
                    error=GRBgetcoeff(model,i,ic,&a);
                    if(a!=0){
                        indexes.push_back(ic);
                        values.push_back(a);
                    }
                }
                std::string name;
                _getRowName(i,name);

                double up;
                error=GRBgetdblattrelement(model,"RHS",i,&up);

                error=GRBdelconstrs(model,1,ra);
                GRBupdatemodel(model);

                if(lo==up){
                    slacks.erase(i);
                    error=GRBaddconstr(model,values.size(),&indexes.front(),&values.front(),GRB_EQUAL,static_cast<double>(up),0);
                }
                else{
                    slacks[i]=std::make_pair(lo,up);
                    error=GRBaddrangeconstr(model,values.size(),&indexes.front(),&values.front(),static_cast<double>(lo),static_cast<double>(up),const_cast<char*>(name.c_str()));
                }
            }
        }

        GRBupdatemodel(model);
    }


    GurobiBase::Value GurobiBase::_getRowLowerBound(int i)const{

        char sense;
        int error=0;
        int ra[2];
        ra[1]=i;

        error=GRBgetcharattrelement(model,"Sense",i,&sense);

        if(sense=='>'){
            double lo;
            error=GRBsetdblattrelement(model,"RHS",i,lo);
            return static_cast<Value>(lo);
        }
        else if(sense=='='){
            if(slacks.find(i)!=slacks.end()){
                return const_cast<std::map<int,std::pair<Value,Value> > *>(&slacks)->operator[](i).first;
            }
            else{
                double lo;
                error=GRBgetdblattrelement(model,"RHS",i,&lo);
                return static_cast<Value>(lo);
            }
        }
        else return -INF;
    }


    void GurobiBase::_setRowUpperBound(int i,Value up){
        LEMON_ASSERT(up!=-INF,"Invalid bound");

        char sense;
        int error=0;
        int ra[2];
        ra[1]=i;

        error=GRBgetcharattrelement(model,"Sense",i,&sense);

        if(sense=='<'){
            error=GRBsetdblattrelement(model,"RHS",i,up);
        }
        else if(sense=='='){
            if(slacks.find(i)==slacks.end()){
                error=GRBsetcharattrelement(model,"Sense",i,'>');
                error=GRBsetdblattrelement(model,"RHS",i,up);
            }
            else{

                std::vector<int> indexes;
                std::vector<double> values;
                int length;
                error=GRBgetintattr(model,"NumVars",&length);
                for(int ic=1;ic<=length;++ic){
                    double a;
                    error=GRBgetcoeff(model,i,ic,&a);
                    if(a!=0){
                        indexes.push_back(ic);
                        values.push_back(a);
                    }
                }
                std::string name;
                _getRowName(i,name);

                double lo;
                error=GRBgetdblattrelement(model,"RHS",i,&lo);

                error=GRBdelconstrs(model,1,ra);
                GRBupdatemodel(model);
                slacks[i]=std::make_pair(lo,up);
                error=GRBaddrangeconstr(model,values.size(),&indexes.front(),&values.front(),static_cast<double>(lo),static_cast<double>(up),const_cast<char*>(name.c_str()));

            }
        }
        else{
            Value lo;
            if(slacks.find(i)!=slacks.end()){
                lo=slacks[i].first;
            }
            else{
                error=GRBsetdblattrelement(model,"RHS",i,lo);
            }

            if(up==lo && slacks.find(i)==slacks.end()){
                    error=GRBsetcharattrelement(model,"Sense",i,'=');
                    error=GRBsetdblattrelement(model,"RHS",i,lo);
            }
            else{
                std::vector<int> indexes;
                std::vector<double> values;
                int length;
                error=GRBgetintattr(model,"NumVars",&length);
                for(int ic=1;ic<=length;++ic){
                    double a;
                    error=GRBgetcoeff(model,i,ic,&a);
                    if(a!=0){
                        indexes.push_back(ic);
                        values.push_back(a);
                    }
                }
                std::string name;
                _getRowName(i,name);

                double lo;
                error=GRBgetdblattrelement(model,"RHS",i,&lo);

                error=GRBdelconstrs(model,1,ra);
                GRBupdatemodel(model);

                if(lo==up){
                    slacks.erase(i);
                    error=GRBaddconstr(model,values.size(),&indexes.front(),&values.front(),GRB_EQUAL,static_cast<double>(up),0);
                }
                else{
                    slacks[i]=std::make_pair(lo,up);
                    error=GRBaddrangeconstr(model,values.size(),&indexes.front(),&values.front(),static_cast<double>(lo),static_cast<double>(up),const_cast<char*>(name.c_str()));
                }
            }
        }

        GRBupdatemodel(model);
    }


    GurobiBase::Value GurobiBase::_getRowUpperBound(int i) const {
        char sense;
        int error=0;
        int ra[2];
        ra[1]=i;

        error=GRBgetcharattrelement(model,"Sense",i,&sense);

        if(sense=='<'){
            double up;
            error=GRBsetdblattrelement(model,"RHS",i,up);
            return static_cast<Value>(up);
        }
        else if(sense=='='){
            if(slacks.find(i)!=slacks.end()){
                //return slacks[i].second;
                return const_cast<std::map<int,std::pair<Value,Value> > *>(&slacks)->operator[](i).second;
            }
            else{
                double up;
                error=GRBgetdblattrelement(model,"RHS",i,&up);
                return static_cast<Value>(up);
            }
        }
        else return +INF;
    }


    /*************************/

    void GurobiBase::_setObjCoeffs(ExprIterator b,ExprIterator e){
        int error=0;
        int length;
        error=GRBgetintattr(model,"NumVars",&length);
        double a;
        for(int i=0;i <=length;++i){
            error=GRBsetdblattrelement(model,"Obj",i,0.0);
        }
        GRBupdatemodel(model);
        for(ExprIterator it=b;it!=e;++it){
            a=static_cast<double>(it->second);
            error=GRBsetdblattrelement(model,"Obj",it->first,a);
        }
        GRBupdatemodel(model);
    }

    void GurobiBase::_getObjCoeffs(InsertIterator b)const{
        int error=0;
        int length;
        error=GRBgetintattr(model,"NumVars",&length);
        double a;
        for(int i=0;i<=length;++i){
            error=GRBgetdblattrelement(model,"Obj",i,&a);
            if(a!=0.0){
                *b=std::make_pair(i,static_cast<Value>(a));
                ++b;
            }
        }
    }

    void GurobiBase::_setObjCoeff(int i,Value obj_coef){
        int error=GRBsetdblattrelement(model,"Obj",i,static_cast<double>(obj_coef));
        //i = 0 means the constant term (shift)
    }

    GurobiBase::Value GurobiBase::_getObjCoeff(int i)const{
        //i = 0 means the constant term (shift)
        //return glp_get_obj_coef(lp, i);
        double a;
        int error=GRBgetdblattrelement(model,"Obj",i,&a);
        return static_cast<Value>(a);
    }

    void GurobiBase::_setSense(GurobiBase::Sense sense){
        int i;
        int error;
        switch(sense){
            case MIN:
                        error=GRBsetintattr(model,"ModelSense",+1);
                        break;
            case MAX:
                        error=GRBsetintattr(model,"ModelSense",-1);
                        break;
        }
    }

    GurobiBase::Sense GurobiBase::_getSense() const {
        int i;
        int error=GRBgetintattr(model,"ModelSense",&i);
        switch(i){
            case +1:
                        return MIN;
            case -1:
                        return MAX;
            default:
                        LEMON_ASSERT(false, "Wrong sense");
                        return GurobiBase::Sense();
        }
    }

    void GurobiBase::_clear(){
        GRBfreemodel(model);
        rows.clear();
        cols.clear();
    }

    void GurobiBase::_messageLevel(MessageLevel level) {
        switch(level){
            case MESSAGE_NOTHING:
                                    _message_level = 0;
                                    break;
            case MESSAGE_ERROR:
                                    _message_level = 1;
                                    break;
            case MESSAGE_WARNING:
                                    _message_level = 1;
                                    break;
            case MESSAGE_NORMAL:
                                    _message_level = 1;
                                    break;
            case MESSAGE_VERBOSE:
                                    _message_level = 1;
                                    break;
        }
    }


    // GurobiLp members

    GurobiLp::GurobiLp():LpBase(),LpSolver(),GurobiBase(){
        presolver(false);
    }

    GurobiLp::GurobiLp(const GurobiLp& other):LpBase(other),LpSolver(other),GurobiBase(other){
        presolver(false);
    }

    GurobiLp* GurobiLp::newSolver() const { return new GurobiLp; }
    GurobiLp* GurobiLp::cloneSolver() const { return new GurobiLp(*this); }

    const char* GurobiLp::_solverName() const { return "GurobiLp"; }

    void GurobiLp::_clear_temporals() {
        _primal_ray.clear();
        _dual_ray.clear();
    }

    GurobiLp::SolveExitStatus GurobiLp::convertStatus(int status){

        switch(status){

            case GRB_OPTIMAL:
            case GRB_INFEASIBLE:
            case GRB_INF_OR_UNBD:
            case GRB_UNBOUNDED:
                                  return SOLVED;

            default:
                                  return UNSOLVED;
            /*
            case GRB_LOADED:
            case GRB_CUTOFF:
            case GRB_ITERATION_LIMIT:
            case GRB_NODE_LIMIT:
            case GRB_TIME_LIMIT:
            case GRB_SOLUTION_LIMIT:
            case GRB_INTERRUPTED:
            case GRB_SUBOPTIMAL:
            case GRB_NUMERIC:
            */
        }

    }

    GurobiLp::SolveExitStatus GurobiLp::_solve(){
        int error=GRBsetintparam(env,"Method",0);
        error=GRBoptimize(model);
        if(error==0){
            int st;
            error=GRBgetintattr(model,"Status",&st);
            return convertStatus(st);
        }
        else
            return UNSOLVED;
    }

    GurobiLp::SolveExitStatus GurobiLp::solvePrimal(){
        int error=GRBsetintparam(env,"Method",1);
        error=GRBoptimize(model);
        if(error==0){
            int st;
            error=GRBgetintattr(model,"Status",&st);
            return convertStatus(st);
        }
        else
            return UNSOLVED;
    }

    GurobiLp::SolveExitStatus GurobiLp::solveDual() {
        int error=GRBsetintparam(env,"Method",2);
        error=GRBoptimize(model);
        if(error==0){
            int st;
            error=GRBgetintattr(model,"Status",&st);
            return convertStatus(st);
        }
        else
            return UNSOLVED;
    }

    GurobiLp::Value GurobiLp::_getPrimal(int i)const{
        double x;
        int error=GRBgetdblattrelement(model,"X",i,&x);
        return static_cast<Value>(x);
    }


    GurobiLp::Value GurobiLp::_getDual(int i)const{
        double y;
        int error=GRBgetdblattrelement(model,"Pi",i,&y);
        return static_cast<Value>(y);
    }

    GurobiLp::Value GurobiLp::_getPrimalValue()const{
        double x;
        int error=GRBgetdblattr(model,"ObjVal",&x);
        return static_cast<Value>(x);
    }

    GurobiLp::VarStatus GurobiLp::_getColStatus(int i)const{
        int y;
        int error=GRBgetintattrelement(model,"VBasis",i,&y);

        switch(y){
            case 0:
                        return BASIC;
            case -1:
                        return LOWER;
            case -2:
                        return UPPER;
            case -3:
                        return FREE;
            default:
                        LEMON_ASSERT(false, "Wrong column status");
                        return GurobiLp::VarStatus();
        }
    }

    GurobiLp::VarStatus GurobiLp::_getRowStatus(int i)const{
        int y;
        int error=GRBgetintattrelement(model,"CBasis",i,&y);

        switch (y){
            case 0:
                        return BASIC;
            case -1:
                        return FREE;
            default:
                        LEMON_ASSERT(false, "Wrong row status");
                        return GurobiLp::VarStatus();
        }
    }

    GurobiLp::Value GurobiLp::_getPrimalRay(int i)const{
        return 22.3;
    }

    GurobiLp::Value GurobiLp::_getDualRay(int i) const {
        return 22.4;
    }

    GurobiLp::ProblemType GurobiLp::_getPrimalType() const {

        int status;
        int error=GRBgetintattr(model,"Status",&status);
        switch(status){

            case GRB_OPTIMAL:
                                  return OPTIMAL;
            case GRB_INFEASIBLE:
                                  return INFEASIBLE;
            case GRB_INF_OR_UNBD:
            case GRB_UNBOUNDED:
                                  return UNBOUNDED;

            default:
                                  return UNDEFINED;
            /*
            case GRB_LOADED:
            case GRB_CUTOFF:
            case GRB_ITERATION_LIMIT:
            case GRB_NODE_LIMIT:
            case GRB_TIME_LIMIT:
            case GRB_SOLUTION_LIMIT:
            case GRB_INTERRUPTED:
            case GRB_SUBOPTIMAL:
            case GRB_NUMERIC:
            */
        }
    }

    GurobiLp::ProblemType GurobiLp::_getDualType()const{
        return _getPrimalType();
    }

    void GurobiLp::presolver(bool presolve) {
        _presolve = presolve;
    }

    // GurobiMip members
    GurobiMip::GurobiMip():LpBase(),MipSolver(),GurobiBase(){
    }

    GurobiMip::GurobiMip(const GurobiMip& other):LpBase(),MipSolver(),GurobiBase(other){
    }

    void GurobiMip::_setColType(int i,GurobiMip::ColTypes col_type){
        int error=0;
        switch(col_type){
            case INTEGER:
                            error=GRBsetcharattrelement(model,"VType",i,'I');
                            break;
            case REAL:
                            error=GRBsetcharattrelement(model,"VType",i,'C');
                            break;
        }
    }

    GurobiMip::ColTypes GurobiMip::_getColType(int i) const {
        int error=0;
        char ch;
        error=GRBgetcharattrelement(model,"VType",i,&ch);
        switch(ch){
            case 'I':
            case 'B':
                        return INTEGER;
            default:
                        return REAL;
        }

    }


    GurobiMip::SolveExitStatus GurobiMip::convertStatus(int status){

        switch(status){

            case GRB_OPTIMAL:
            case GRB_INFEASIBLE:
            case GRB_INF_OR_UNBD:
            case GRB_UNBOUNDED:
                                  return SOLVED;

            default:
                                  return UNSOLVED;
            /*
            case GRB_LOADED:
            case GRB_CUTOFF:
            case GRB_ITERATION_LIMIT:
            case GRB_NODE_LIMIT:
            case GRB_TIME_LIMIT:
            case GRB_SOLUTION_LIMIT:
            case GRB_INTERRUPTED:
            case GRB_SUBOPTIMAL:
            case GRB_NUMERIC:
            */
        }

    }

    GurobiMip::SolveExitStatus GurobiMip::_solve(){
        int error=GRBoptimize(model);
        if(error==0){
            int st;
            error=GRBgetintattr(model,"Status",&st);
            return convertStatus(st);
        }
        else
            return UNSOLVED;
    }


    GurobiMip::ProblemType GurobiMip::_getType() const {
        int status;
        int error=GRBgetintattr(model,"Status",&status);
        switch(status){

            case GRB_OPTIMAL:
                                  return OPTIMAL;
            case GRB_INFEASIBLE:
                                  return INFEASIBLE;
            case GRB_INF_OR_UNBD:
            case GRB_UNBOUNDED:
                                  return UNBOUNDED;

            default:
                                  return UNDEFINED;
            /*
            case GRB_LOADED:
            case GRB_CUTOFF:
            case GRB_ITERATION_LIMIT:
            case GRB_NODE_LIMIT:
            case GRB_TIME_LIMIT:
            case GRB_SOLUTION_LIMIT:
            case GRB_INTERRUPTED:
            case GRB_SUBOPTIMAL:
            case GRB_NUMERIC:
            */
    }
  }

    GurobiMip::Value GurobiMip::_getSol(int i)const{
        double x;
        int error=GRBgetdblattrelement(model,"X",i,&x);
        return static_cast<Value>(x);
    }

    GurobiMip::Value GurobiMip::_getSolValue()const{
        double x;
        int error=GRBgetdblattr(model,"ObjVal",&x);
        return static_cast<Value>(x);
    }

    GurobiMip* GurobiMip::newSolver() const { return new GurobiMip; }
    GurobiMip* GurobiMip::cloneSolver() const {return new GurobiMip(*this); }

    const char* GurobiMip::_solverName() const { return "GurobiMip"; }

} //END OF NAMESPACE LEMON
