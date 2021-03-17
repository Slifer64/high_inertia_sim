#include <cmath>
#include <armadillo>

arma::mat lwr4p_inertia(const arma::vec &q);

arma::mat lwr4p_coriolis(const arma::vec &q, const arma::vec &dq);

arma::vec lwr4p_gravity(const arma::vec &q);

arma::vec lwr4p_friction(const arma::vec &dq);

arma::mat lwr4p_fkine(const arma::vec &q);

arma::mat lwr4p_jacob(const arma::vec &q);
