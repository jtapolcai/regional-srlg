 //==============================================================
/// \file       : make_names.h
/// \brief This header contains debugging functions for lemon
 //--------------------------------------------------------------
/// \author              Gabor Nemeth and Janos Tapolcai
/// \author              ng201@mail2.selye.sk, tapolcai@tmit.bme.hu
/// \date                August 15-22, 2005
 //==============================================================

#ifndef _LEMON_MAKE_NAMES_H_
#define _LEMON_MAKE_NAMES_H_

#include <lemon/core.h>

namespace lemon{

#ifdef LATEX_LOG
    template<typename Digraph>
    inline const std::string arc_name(const typename Digraph::Arc & a, const Digraph & g)
    {
        std::ostringstream streamOut;
        streamOut <<"$e_{"<< g.id(g.source(a)) <<"\\rightarrow "<< g.id(g.target(a))<<"}$";
        return streamOut.str();
    }
    template< typename Digraph>
    inline const std::string node_name(const typename Digraph::Node & n, const Digraph & g)
    {
        std::ostringstream streamOut;
        streamOut << "$n_{" <<g.id(n)<<"}$";
        return streamOut.str();
    }
    template< typename Graph>
    inline const std::string edge_name(const typename Graph::Edge & e, const Graph & g){
        std::ostringstream streamOut;
        streamOut <<"$e_{"<< g.id(g.u(e)) <<","<< g.id(g.v(e))<<"}$";
        return streamOut.str();
    }
#else
    template<typename Digraph>
    inline const std::string arc_name(const typename Digraph::Arc & a, const Digraph & g)
    {
        std::ostringstream streamOut;
        streamOut <<"Arc n"<< g.id(g.source(a)) <<"->n"<< g.id(g.target(a));
        return streamOut.str();
    }
    template< typename Digraph>
    inline const std::string node_name(const typename Digraph::Node & n, const Digraph & g)
    {
        std::ostringstream streamOut;
        streamOut << "Node n" <<g.id(n);
        return streamOut.str();
    }
    template< typename Graph>
    inline const std::string edge_name(const typename Graph::Edge & e, const Graph & g){
        std::ostringstream streamOut;
        streamOut <<"Edge n"<< g.id(g.u(e)) <<"-n"<< g.id(g.v(e));
        return streamOut.str();
    }
#endif
    /// Generates a name (std::string) for an Arc.
		template< typename type, typename Digraph>
		inline const std::string make_name( const type & name, const typename Digraph::Arc & e, const Digraph & g)
		{
			std::ostringstream streamOut;
			streamOut << g.id(g.source(e)) << name << g.id(g.target(e)); 
			return streamOut.str();
		}
		template< typename type, typename Digraph>
		inline const std::string make_name(const typename Digraph::Arc & e, const Digraph & g){
			return make_name("->",e,g);
		}

		/// Generates a name (std::string) for an Arc.
		template< typename type, typename Digraph>
		inline const std::string make_name( const type & name, const typename Digraph::Edge & e, const Digraph & g)
		{
			std::ostringstream streamOut;
			streamOut << g.id(g.u(e)) << name << g.id(g.v(e)); 
			return streamOut.str();
		}
		template< typename type, typename Digraph>
		inline const std::string make_name(const typename Digraph::Edge & e, const Digraph & g){
			return make_name("-",e,g);
		}
    
    /// Generates a name (std::string) for an Arc.
    
    /// Genarates a name (std::string) for a Node.
		template< typename Digraph>
		inline const std::string make_name(const typename Digraph::Node & n, const Digraph & g)
		{
			std::ostringstream streamOut;
			streamOut << "n" <<g.id(n); 
			return streamOut.str();
		}
		
		/// Generates a name (std::string) for a SRG.
		template< typename SRGSet >
		inline const std::string make_name(const typename SRGSet::SRG & s, const SRGSet & srg_set)
		{
			std::ostringstream streamOut;
			streamOut << "srg" << srg_set.id(s); 
			return streamOut.str();
		}
		
		template< typename SRGSet, typename Digraph >
		inline const std::string print_srg(const typename SRGSet::SRG & s, const SRGSet & srg_set, const Digraph & g)
		{
			std::ostringstream streamOut;
			for(typename SRGSet::SRGArcIt e(srg_set,s); e != lemon::INVALID;++e)
			{
				streamOut << make_name("->",e,g)<<" ";
			}
			return streamOut.str();
		}
		
		/// Generates a name (std::string) for an Arc.
		template< typename type>
		inline const std::string make_name(const type & name, int id)
		{
			std::ostringstream streamOut;
			streamOut << name << id; 
			return streamOut.str();
		}
	
		template< typename type>
		inline const std::string make_name(const type & name, double id)
		{
			std::ostringstream streamOut;
			streamOut << name << id; 
			return streamOut.str();
		}

#ifdef MAP_LOG
#include <lemon/maps.h>
	
template<typename M, typename Graph>
  class LoggerMap : public MapBase<typename M::Key, typename M::Value> {
    M _m;
    const Graph & _g;
    int level;
    std::string name;    
  public:
    ///\e
    typedef typename M::Key Key;
    ///\e
    typedef typename M::Value Value;
	typedef typename M::Reference Reference;

    /// Constructor

    /// Constructor.
    /// \param m The undelying map.
    /// \param v The constant value.
    LoggerMap(const Graph & g, int _level=2, std::string _name="") : _m(g), level(_level), name(_name), _g(g) {}
    ///\e
    Value operator[](const Key &k) const { return _m[k]; }
    ///\e
    void set(const Key &k, const Value &v) {		
      std::cout<<"\nmap "<<name<<"["<<make_name(k,_g)<<"]="<<v;
	  _m.set(k, v); 		
	}
	      /// Returns a reference to the value associated with the given key.
    Reference operator[](const Key &k) {
    	std::cout<<"\nmap "<<name<<"["<<make_name(k,_g)<<"]=?="<<_m[k];	
        return _m.operator[](k);
    }
  };
    
#endif
    
}; //namespace

#endif
