

arma::vec lwr4p_gravity(const arma::vec &q)
{
  arma::vec sinq(7), cosq(7);

  for (int i=0; i<7; i++)
  {
    cosq(i) = std::cos(q(i));
    sinq(i) = std::sin(q(i));
  }

  arma::vec G(7);

   G(0) = 0;
   G(1) = 981*(-(-0.0390675737668488*sinq(4)*sinq(5) + 0.0327196625084711*cosq(4))*sinq(2) + ((0.0390675737668488*cosq(5) + 1.3011637173221)*sinq(3) + (-0.0327196625084711*sinq(4) - 0.0390675737668488*sinq(5)*cosq(4) - 0.0220471479660755)*cosq(3) + 0.0283274807515703)*cosq(2))*cosq(1)/100 - 981*((0.0390675737668488*cosq(5) + 1.3011637173221)*cosq(3) - (-0.0327196625084711*sinq(4) - 0.0390675737668488*sinq(5)*cosq(4) - 0.0220471479660755)*sinq(3) + 3.33001322499426)*sinq(1)/100;
   G(2) = -981*(-0.0390675737668488*sinq(4)*sinq(5) + 0.0327196625084711*cosq(4))*sinq(1)*cosq(2)/100 - 981*((0.0390675737668488*cosq(5) + 1.3011637173221)*sinq(3) + (-0.0327196625084711*sinq(4) - 0.0390675737668488*sinq(5)*cosq(4) - 0.0220471479660755)*cosq(3) + 0.0283274807515703)*sinq(1)*sinq(2)/100;
   G(3) = -981*((0.0390675737668488*cosq(5) + 1.3011637173221)*sinq(3) + (-0.0327196625084711*sinq(4) - 0.0390675737668488*sinq(5)*cosq(4) - 0.0220471479660755)*cosq(3))*cosq(1)/100 + 981*((0.0390675737668488*cosq(5) + 1.3011637173221)*cosq(3) - (-0.0327196625084711*sinq(4) - 0.0390675737668488*sinq(5)*cosq(4) - 0.0220471479660755)*sinq(3))*sinq(1)*cosq(2)/100;
   G(4) = 981*(0.0390675737668488*sinq(4)*sinq(5) - 0.0327196625084711*cosq(4))*sinq(1)*cosq(2)*cosq(3)/100 - 981*(-0.0390675737668488*sinq(1)*sinq(2)*sinq(5) - 0.0327196625084711*sinq(3)*cosq(1))*cosq(4)/100 + 0.320979889208102*sinq(1)*sinq(2)*sinq(4) - 0.383252898652787*sinq(3)*sinq(4)*sinq(5)*cosq(1);
   G(5) = -981*(0.0390675737668488*sinq(3)*sinq(5) + 0.0390675737668488*cosq(3)*cosq(4)*cosq(5))*sinq(1)*cosq(2)/100 + 981*(0.0390675737668488*sinq(3)*cosq(4)*cosq(5) - 0.0390675737668488*sinq(5)*cosq(3))*cosq(1)/100 + 0.383252898652787*sinq(1)*sinq(2)*sinq(4)*cosq(5);
   G(6) = 0;

   return G;
}
