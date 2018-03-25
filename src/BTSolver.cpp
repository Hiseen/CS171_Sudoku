#include"BTSolver.hpp"

using namespace std;

// =====================================================================
// Constructors
// =====================================================================

BTSolver::BTSolver ( SudokuBoard input, Trail* _trail,  string val_sh, string var_sh, string cc )
: sudokuGrid( input.get_p(), input.get_q(), input.get_board() ), network( input )
{
	p = input.get_p();
	q = input.get_q();
	n = p*q;
	valHeuristics = val_sh;
	varHeuristics = var_sh;
	cChecks =  cc;
	trail = _trail;
}

// =====================================================================
// Consistency Checks
// =====================================================================

// Basic consistency check, no propagation done
bool BTSolver::assignmentsCheck ( void )
{
	for ( Constraint c : network.getConstraints() )
		if ( ! c.isConsistent() )
			return false;

	return true;
}

/**
 * Part 1 TODO: Implement the Forward Checking Heuristic
 *
 * This function will do both Constraint Propagation and check
 * the consistency of the network
 *
 * (1) If a variable is assigned then eliminate that value from
 *     the square's neighbors.
 *
 * Note: remember to trail.push variables before you change their domain
 * Return: true is assignment is consistent, false otherwise
 */
bool BTSolver::forwardChecking ( void )
{
	for (auto i : network.getVariables())
	{
		if (i->isAssigned())
		{
			if (!RecursiveFC(i))return false;
			/*
			auto val = i->getAssignment();
			for (auto j : network.getNeighborsOfVariable(i))
			{
				if (j->getDomain().contains(val))
				{
					trail->push(j);
					j->removeValueFromDomain(val);
				}
				if (j->size() == 0)return false;
			}
			*/
		}
	}
	return true;
}

/**
 * Part 2 TODO: Implement both of Norvig's Heuristics
 *
 * This function will do both Constraint Propagation and check
 * the consistency of the network
 *
 * (1) If a variable is assigned then eliminate that value from
 *     the square's neighbors.
 *
 * (2) If a constraint has only one possible place for a value
 *     then put the value there.
 *
 * Note: remember to trail.push variables before you change their domain
 * Return: true is assignment is consistent, false otherwise
 */
bool BTSolver::norvigCheck ( void )
{
	//(1)
	for (auto i : network.getVariables())
	{
		if (i->isAssigned())
		{
			if (!RecursiveFC(i))return false;
		}
	}
	//(2)
	for (auto i : network.getConstraints())
	{
		map<int, vector<Variable*>> m;
		for (auto j : i.vars)
		{
			for (auto k : j->getValues())
				m[k].push_back(j);
		}
		for (auto j : m)
		{
			if (j.second.size() == 1 && !(j.second[0]->isAssigned()))
			{
				trail->push(j.second[0]);
				j.second[0]->assignValue(j.first);
				if (!RecursiveFC(j.second[0]))return false;
			}
		}
	}
	return true;
}

void BTSolver::NakedSubset()
{
	for (auto i : network.getConstraints())
	{
		for (auto j : i.vars)
		{
			if (j->isAssigned())continue;
			unordered_set<Variable*> s(n);
			s.insert(j);
			for (auto k : i.vars)
			{
				if (j != k && !(k->isAssigned()) && IsSubset(j->getValues(), k->getValues()))
					s.insert(k);
				if (s.size() == j->size())break;
			}
			if (s.size() == j->size())
			{
				for (auto k : i.vars)
				{
					if (!(k->isAssigned()) && s.find(k) == s.end() && Intersects(k->getValues(), j->getValues()))
					{
						trail->push(k);
						k->setDomain(Domain(Substraction(k->getValues(), j->getValues())));
					}
				}
			}
		}
	}
}


bool BTSolver::HiddenSubset()
{
	for (auto i : network.getConstraints())
	{
		map<int, vector<Variable*>> m;
		for (auto j : i.vars)
		{
			if (!(j->isAssigned()))
			{
				for (auto k : j->getValues())
					m[k].push_back(j);
			}
		}
		unordered_set<int> checked;
		for (auto c : m)
		{
			if (c.second.size() == 1)
			{
				trail->push(c.second[0]);
				c.second[0]->assignValue(c.first);
				if (!RecursiveFC(c.second[0]))return false;
			}
			else if (checked.find(c.first) == checked.end())
			{
				vector<int> remained;
				remained.push_back(c.first);
				checked.insert(c.first);
				for (auto j : m)
				{
					if (j != c && IsEqual(c.second, j.second))
					{
						checked.insert(j.first);
						remained.push_back(j.first);
						if (remained.size() == c.second.size())
						{
							for (auto p : c.second)
							{
								trail->push(p);
								p->setDomain(Domain(remained));
							}

						}
					}
				}
			}
		}
	}
	return true;

}


bool BTSolver::RecursiveFC(Variable * v)
{
	auto val = v->getAssignment();
	for (auto j : network.getNeighborsOfVariable(v))
	{
		if (j->getDomain().contains(val))
		{
			trail->push(j);
			j->removeValueFromDomain(val);
			if (j->size() == 1 && !RecursiveFC(j))
				return false;
			else if (j->size() == 0)
				return false;
		}
	}
	return true;
}

bool BTSolver::AdvancedFC()
{
	for (auto i : network.getVariables())
		if (i->isAssigned() && !RecursiveFC(i))return false;
	return true;
}




template<class T>
bool BTSolver::IsEqual(const vector<T>& a, const vector<T>& b)
{
	if(a.size()!=b.size())return false;
	unordered_set<T> s;
	for (auto i : a)
		s.insert(i);
	for (auto i : b)
		if (s.find(i) == s.end())return false;
	return true;
}


bool BTSolver::Intersects(const Domain::ValueSet& a, const Domain::ValueSet& b)
{
	unordered_set<int> s;
	for (auto i : a)
		s.insert(i);
	for (auto i : b)
		if (s.find(i) != s.end())return true;
	return false;
}



vector<int> BTSolver::Substraction(const Domain::ValueSet& big, const Domain::ValueSet& small)
{
	unordered_set<int> s;
	for (auto i : big)
		s.insert(i);
	for (auto i : small)
		if (s.find(i) != s.end())s.erase(i);
	vector<int> vec;
	vec.insert(vec.end(), s.begin(), s.end());
	return vec;
}



bool BTSolver::IsSubset(const Domain::ValueSet& big, const Domain::ValueSet& small)
{
	if (big.size() < small.size())return false;
	unordered_set<int> s;
	for (auto i : big)
		s.insert(i);
	for (auto i : small)
		if (s.find(i) == s.end())return false;
	return true;
}





/**
 * Optional TODO: Implement your own advanced Constraint Propagation
 *
 * Completing the three tourn heuristic will automatically enter
 * your program into a tournament.
 */
bool BTSolver::getTournCC ( void )
{
	NakedSubset();
	return AdvancedFC() && HiddenSubset();
}

// =====================================================================
// Variable Selectors
// =====================================================================

// Basic variable selector, returns first unassigned variable
Variable* BTSolver::getfirstUnassignedVariable ( void )
{
	for ( Variable* v : network.getVariables() )
		if ( !(v->isAssigned()) )
			return v;

	// Everything is assigned
	return nullptr;
}

/**
 * Part 1 TODO: Implement the Minimum Remaining Value Heuristic
 *
 * Return: The unassigned variable with the smallest domain
 */
Variable* BTSolver::getMRV ( void )
{
	int m = INT_MAX;
	Variable* res = nullptr;
	for (auto i : network.getVariables())
	{
		if (!i->isAssigned() && i->size()<m)
		{
			m = i->size();
			res = i;
		}
	}
	return res;
}

/**
 * Part 2 TODO: Implement the Degree Heuristic
 *
 * Return: The unassigned variable with the most unassigned neighbors
 */
Variable* BTSolver::getDegree ( void )
{
	int m = -1;
	Variable* res = nullptr;
	for (auto i : network.getVariables())
	{
		if (!i->isAssigned())
		{
			int count = 0;
			for (auto j : network.getNeighborsOfVariable(i))
				if (!j->isAssigned())++count;
			if (count > m)
			{
				m = count;
				res = i;
			}
		}
	}
	return res;
}

/**
 * Part 2 TODO: Implement the Minimum Remaining Value Heuristic
 *                with Degree Heuristic as a Tie Breaker
 *
 * Return: The unassigned variable with the smallest domain and involved
 *             in the most constraints
 */
Variable* BTSolver::MRVwithTieBreaker ( void )
{
	int minimum = INT_MAX;
	Variable* res = nullptr;
	int pre_count = 0;
	for (auto i : network.getVariables())
	{
		if (!i->isAssigned())
		{
			if (i->size() < minimum)
			{
				minimum = i->size();
				res = i;
				pre_count = 0;
				for (auto j : network.getNeighborsOfVariable(i))
					if (!j->isAssigned())++pre_count;
			}
			else if (i->size() == minimum)
			{
				int count = 0;
				for (auto j : network.getNeighborsOfVariable(i))
					if (!j->isAssigned())++count;
				if (count >= pre_count)
				{
					res = i;
					pre_count = count;
				}
			}
		}
	}
	return res;
}

/**
 * Optional TODO: Implement your own advanced Variable Heuristic
 *
 * Completing the three tourn heuristic will automatically enter
 * your program into a tournament.
 */
Variable* BTSolver::getTournVar ( void )
{
	return MRVwithTieBreaker();
}

// =====================================================================
// Value Selectors
// =====================================================================

// Default Value Ordering
vector<int> BTSolver::getValuesInOrder ( Variable* v )
{
	vector<int> values = v->getDomain().getValues();
	sort( values.begin(), values.end() );
	return values;
}

/**
 * Part 1 TODO: Implement the Least Constraining Value Heuristic
 *
 * The Least constraining value is the one that will knock the least
 * values out of it's neighbors domain.
 *
 * Return: A list of v's domain sorted by the LCV heuristic
 *         The LCV is first and the MCV is last
 */
vector<int> BTSolver::getValuesLCVOrder ( Variable* v )
{
	map<int, int> temp;
	for (auto i : v->getValues())
		temp[i] = 0;
	for (auto i : network.getNeighborsOfVariable(v))
	{
		for (auto j : i->getValues())
		{
			if (temp.find(j) != temp.end())
				temp[j] += 1;
		}
	}
	std::vector<pair<int, int>> vec;
	for (auto i : temp)
		vec.push_back(make_pair(i.second, i.first));
	std::sort(vec.begin(), vec.end());
	vector<int> res;
	for (auto i : vec)
		res.push_back(i.second);
	return res;
}

/**
 * Optional TODO: Implement your own advanced Value Heuristic
 *
 * Completing the three tourn heuristic will automatically enter
 * your program into a tournament.
 */
vector<int> BTSolver::getTournVal ( Variable* v )
{
	return getValuesLCVOrder(v);
}

// =====================================================================
// Engine Functions
// =====================================================================

void BTSolver::solve ( void )
{
	if ( hasSolution )
		return;

	// Variable Selection
	Variable* v = selectNextVariable();

	if ( v == nullptr )
	{
		for ( Variable* var : network.getVariables() )
		{
			// If all variables haven't been assigned
			if ( ! ( var->isAssigned() ) )
			{
				cout << "Error" << endl;
				return;
			}
		}

		// Success
		hasSolution = true;
		return;
	}

	// Attempt to assign a value
	for ( int i : getNextValues( v ) )
	{
		// Store place in trail and push variable's state on trail
		trail->placeTrailMarker();
		trail->push( v );

		// Assign the value
		v->assignValue( i );

		// Propagate constraints, check consistency, recurse
		if ( checkConsistency() )
			solve();

		// If this assignment succeeded, return
		if ( hasSolution )
			return;

		// Otherwise backtrack
		trail->undo();
	}
}

bool BTSolver::checkConsistency ( void )
{
	if ( cChecks == "forwardChecking" )
		return forwardChecking();

	if ( cChecks == "norvigCheck" )
		return norvigCheck();

	if ( cChecks == "tournCC" )
		return getTournCC();

	return assignmentsCheck();
}

Variable* BTSolver::selectNextVariable ( void )
{
	if ( varHeuristics == "MinimumRemainingValue" )
		return getMRV();

	if ( varHeuristics == "Degree" )
		return getDegree();

	if ( varHeuristics == "MRVwithTieBreaker" )
		return MRVwithTieBreaker();

	if ( varHeuristics == "tournVar" )
		return getTournVar();

	return getfirstUnassignedVariable();
}

vector<int> BTSolver::getNextValues ( Variable* v )
{
	if ( valHeuristics == "LeastConstrainingValue" )
		return getValuesLCVOrder( v );

	if ( valHeuristics == "tournVal" )
		return getTournVal( v );

	return getValuesInOrder( v );
}

bool BTSolver::haveSolution ( void )
{
	return hasSolution;
}

SudokuBoard BTSolver::getSolution ( void )
{
	return network.toSudokuBoard ( sudokuGrid.get_p(), sudokuGrid.get_q() );
}

ConstraintNetwork BTSolver::getNetwork ( void )
{
	return network;
}
