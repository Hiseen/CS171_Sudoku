#ifndef BTSOLVER_HPP
#define BTSOLVER_HPP

#include "SudokuBoard.hpp"
#include "Domain.hpp"
#include "Variable.hpp"
#include "ConstraintNetwork.hpp"
#include "Trail.hpp"

#include <utility>
#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_map>

#ifndef INT_MAX
#define INT_MAX 2147483647
#endif

using namespace std;

class BTSolver
{
public:
	// Constructor
	BTSolver ( SudokuBoard board, Trail* trail, std::string val_sh, std::string var_sh, std::string cc );

	// Consistency Checks (Implement these)
	bool assignmentsCheck ( void );
	bool forwardChecking  ( void );
	bool norvigCheck      ( void );
	bool getTournCC       ( void );

	// Variable Selectors (Implement these)
	Variable* getfirstUnassignedVariable ( void );
	Variable* getMRV            ( void );
	Variable* getDegree         ( void );
	Variable* MRVwithTieBreaker ( void );
	Variable* getTournVar       ( void );

	// Value Selectors (Implement these)
	std::vector<int> getValuesInOrder  ( Variable* v );
	std::vector<int> getValuesLCVOrder ( Variable* v );
	std::vector<int> getTournVal       ( Variable* v );

	// Engine Functions
	void solve ( void );

	bool checkConsistency ( void );
	Variable* selectNextVariable ( void );
	std::vector<int> getNextValues ( Variable* v );

	// Helper Functions
	bool haveSolution ( void );
	SudokuBoard getSolution ( void );
	ConstraintNetwork getNetwork ( void );

private:
	// Properties
	ConstraintNetwork network;
	SudokuBoard sudokuGrid;
	Trail* trail;

	bool hasSolution = false;

	std::string varHeuristics;
	std::string valHeuristics;
	std::string cChecks;

	int p, q, n;
	void NakedSubset();
	bool HiddenSubset();
	bool AdvancedFC();
	bool RecursiveFC(Variable* v);
	template<class T>
	bool IsEqual(const vector<T>& a, const vector<T>& b);
	bool IsSubset(const Domain::ValueSet& big, const Domain::ValueSet& small);
	bool Intersects(const Domain::ValueSet& a, const Domain::ValueSet& b);
	vector<int> Substraction(const Domain::ValueSet& big, const Domain::ValueSet& small);
};


#endif

