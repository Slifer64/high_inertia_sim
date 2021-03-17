

arma::vec lwr4p_friction(const arma::vec &dq)
{
  arma::vec Fr(7);

  Fr(0) = 0.183379012548875*std::atan(100.0*dq(0));
  Fr(1) = 0.173198345026047*std::atan(100.0*dq(1));
  Fr(2) = 0.27004148667018*std::atan(100.0*dq(2));
  Fr(3) = 0.203723283726169*std::atan(100.0*dq(3));
  Fr(4) = 0.128735665726732*std::atan(100.0*dq(4));
  Fr(5) = 0.0210453117134514*std::atan(100.0*dq(5));
  Fr(6) = 0.0826258245130735*std::atan(100.0*dq(6));

  return Fr;
}
