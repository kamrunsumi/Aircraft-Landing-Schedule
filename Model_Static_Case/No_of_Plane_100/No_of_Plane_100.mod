/*********************************************
 * OPL 12.8.0.0 Model
 * Author: Kamrun
 * Creation Date: Nov 11, 2019 at 10:04:29 PM
 *********************************************/
///Scheduling Aircraft Landing.Single runway formulation (Static Case)
///Landing times of aircrafts are determined satisfying certain constraints
///(1)The aircraft should land within a predetermined time interval
///(2)Separation time between two landing should be satisfied

///Notation
//Let

int i = 100;//No of planes
range I= 1..i;
range J= 1..i;


///Parameters
int E[I] = ...; // Earliest allowable landing time for plane i.
int L[I] = ...; //Latest allowable landing time for plane i.
int T[I] = ...; //Target landing time for plane i.
int S[I][J] = ...; //The required separation time(>=0) between plane i landing and plane j landing (where plane i land before plane j)
int g[I] = ...; // The penalty cost (>=0) per unit of time for landing before the target time T[I] for plane i = (1,...P)
int h[I] = ...; // The penalty cost (>=0) per unit of time for landing after the target time T[I] for plane i = (1,...P)
int M = ...; // Big number //M=max(L[1,:])-min(E[1,:]); 




// Decision variables
dvar float+ x[I] in 0..infinity ; //(Using continious decision variable) The landing time for plane i.
dvar float+ alpha[I]in 0..infinity; //(Using continious decision variable) how soon plane i lands before T[i]
dvar float+ beta[I]in 0..infinity; //(Using continious decision variable) How soon plane i lands after T[i]
dvar boolean delta[I][J] in 0..1; // 1 if plane i lands before plane j; otherwise 0
 
 
 
/// Objective function(MIP)
 minimize 
 sum(i in I) (g[i]*alpha[i] + h[i]*beta[i]);
 
//Depending on which plane lands before what and whether separation constraint is satisfied or 
// not, there are three situations observed.
//U= the set of pais (i,j) of planes for which we are uncertain whether plane i lands before 
// plane j.
//V= the set of pairs (i,j) of planes for which i definitely lands before j (but for which
// separation constraint is not automatically satisfied)
//W= the set of pais (i,j) of planes for which i definitley lands before j (and for which 
//the separation constraint is automatically satisfied)
 
/// Constraints
 subject to {
// 1.Either plane i must land before plane j(delta[i][j]=1) or plane j must land before plane i(delta[j][i]=1) 
Cns1: forall(i in I, j in J)
if (j!=i) delta[i][j]+delta[j][i]==1; 
 else  delta[i][j]==0;



//2. landing time for each plane should be equal or greater than the earliest allowable landing time.This 
//constrain ensure each plane lands within its time window.  
 Cns2: forall(i in I) 
 x[i]>=E[i];
 
 
//3. landing time for each plane should be equal or less than the latest allowable landing time.This constraint
//ensure that each plane lands within its time window. 
 Cns3: forall(i in I) 
  x[i]<=L[i];
 
 
//4 & 5. Ensure that  alpha[i] is at least as big as zero and the time difference between  T[i] and x[i], and at 
// most the time difference between T[i] and E[i].
 Cns4: forall(i in I)
  alpha[i]>= T[i] - x[i];
 
 Cns5:  0<= alpha[i] <= T[i] - E[i];
 
 
//6 & 7. Ensure that  beta[i] is at least as big as zero and the time difference between  x[i] and T[i], and at 
// most the time difference between L[i] and T[i].
 Cns6: forall(i in I)
  beta[i]>= x[i] - T[i];
  
 Cns7: 
  0<= beta[i] <= L[i] - T[i];

 // 8.Impose the separation time constraint when j lands before i.
  
 Cns8: forall(i in I, j in J) 
  if (j!=i)   x[j] - x[i] >= S[i][j] - M*delta[j][i]; 
  
} 

 
execute pp
{ 

           writeln("The landing time for plane i: ", x);
           writeln("How soon plane i lands before T[i]:", alpha);
           writeln("How soon plane i lands after T[i]:", beta);
           writeln("1 if plane i lands before plane j; otherwise 0:", delta)

}
 
main  

        {
      var opl = thisOplModel;
      opl.generate();
          cplex.exportModel("Aircraft_landing.lp");
          cplex.solve();
          writeln("Obj Value: ", cplex.getObjValue());
          opl.postProcess(); 
        }                   
        



 
 

