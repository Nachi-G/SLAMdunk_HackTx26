// Persistent simulator RPC worker. Never addresses or moves x500_0.
#include <gz/transport/Node.hh>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/entity.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>

int main(int argc,char **argv) {
  bool slalom=false,challenge=false;
  for(int j=1;j<argc;j++){slalom |= std::string(argv[j])=="--slalom";challenge |= std::string(argv[j])=="--challenge";}
  const int count=challenge?12:(slalom?3:2);
  const double xs_baseline[12]={4.0,8.0};
  const double xs_slalom[12]={2.8,6.5,10.5};
  const double xs_challenge[12]={2.8,6.5,10.5,0.0,4.0,8.0,12.0,0.0,4.0,8.0,12.0,16.0};
  const double *xs=challenge?xs_challenge:(slalom?xs_slalom:xs_baseline);
  const double centers_baseline[12]={0.0,0.0};
  const double centers_slalom[12]={0.0,2.7,-2.2};
  const double centers_challenge[12]={0.0,2.7,-2.2,6.0,6.0,6.0,6.0,-6.0,-6.0,-6.0,-6.0,0.0};
  const double *centers=challenge?centers_challenge:(slalom?centers_slalom:centers_baseline);
  const double amplitudes_baseline[12]={2.5,2.8};
  const double amplitudes_slalom[12]={0.35,0.35,0.35};
  const double amplitudes_challenge[12]={0.35,0.35,0.35,0.55,0.55,0.55,0.55,0.55,0.55,0.55,0.55,0.55};
  const double *amplitudes=challenge?amplitudes_challenge:(slalom?amplitudes_slalom:amplitudes_baseline);
  const double frequencies_baseline[12]={0.28,0.24};
  const double frequencies_slalom[12]={0.45,0.4,0.42};
  const double frequencies_challenge[12]={0.45,0.4,0.42,0.3,0.325,0.35,0.375,0.4,0.425,0.45,0.475,0.5};
  const double *frequencies=challenge?frequencies_challenge:(slalom?frequencies_slalom:frequencies_baseline);
  const double phases_baseline[12]={0.0,1.7};
  const double phases_slalom[12]={0.0,1.7,0.8};
  const double phases_challenge[12]={0.0,1.7,0.8,0.0,0.7,1.4,2.0999999999999996,2.8,3.5,4.199999999999999,4.8999999999999995,5.6};
  const double *phases=challenge?phases_challenge:(slalom?phases_slalom:phases_baseline);
  for(int j=1;j<argc;j++) {
    if(std::string(argv[j])=="--describe") {
      std::cout<<std::setprecision(12);
      for(int i=0;i<count;i++)std::cout<<xs[i]<<" "<<centers[i]<<" "<<amplitudes[i]<<" "<<frequencies[i]<<" "<<phases[i]<<" .65"<<std::endl;
      return 0;
    }
  }
  gz::transport::Node node;
  std::string names[12];
  for(int i=0;i<12;i++)names[i]="quick_demo_crossing_"+std::to_string(i+1);
  std::string line;
  while(std::getline(std::cin,line)) {
    std::istringstream input(line);char operation=' ';input>>operation;
    bool executed=false,service_result=false;gz::msgs::Boolean response;
    if(operation=='C') {
      int i=-1;input>>i;
      if(i<0||i>=count){std::cout<<"ERROR invalid obstacle index"<<std::endl;continue;}
      const double x=xs[i],y=centers[i]+amplitudes[i]*std::sin(phases[i]);
      const std::string color=i==0?"1 .15 .1 1":(i==1?".1 .45 1 1":".1 .9 .3 1");
      std::ostringstream xml;
      xml<<"<sdf version='1.9'><model name='"<<names[i]<<"'><static>true</static><pose>"<<x<<" "<<y<<" 2 0 0 0</pose>"
         <<"<link name='obstacle'><collision name='body'><geometry><cylinder><radius>.65</radius><length>3</length></cylinder></geometry></collision>"
         <<"<sensor name='demo_contact' type='contact'><always_on>true</always_on><update_rate>30</update_rate><topic>/demo/contacts</topic><contact><collision>body</collision></contact></sensor>"
         <<"<visual name='body'><geometry><cylinder><radius>.65</radius><length>3</length></cylinder></geometry><material><ambient>"<<color<<"</ambient><diffuse>"<<color<<"</diffuse></material></visual></link></model></sdf>";
      gz::msgs::EntityFactory request;request.set_sdf(xml.str());request.set_name(names[i]);request.set_allow_renaming(false);
      executed=node.Request("/world/default/create",request,4000,response,service_result);
    } else if(operation=='M') {
      double t=0;input>>t;
      if(!std::isfinite(t)||t<0){std::cout<<"ERROR invalid motion time"<<std::endl;continue;}
      gz::msgs::Pose_V request;
      for(int i=0;i<count;i++) {
        auto p=request.add_pose();p->set_name(names[i]);
        p->mutable_position()->set_x(xs[i]);
        p->mutable_position()->set_y(centers[i]+amplitudes[i]*std::sin(frequencies[i]*t+phases[i]));
        p->mutable_position()->set_z(2.);p->mutable_orientation()->set_w(1.);
      }
      executed=node.Request("/world/default/set_pose_vector",request,300,response,service_result);
    } else if(operation=='R') {
      int i=-1;input>>i;
      if(i<2||i>=12){std::cout<<"ERROR only the extra demo obstacle can be removed"<<std::endl;continue;}
      gz::msgs::Entity request;request.set_name(names[i]);request.set_type(gz::msgs::Entity::MODEL);
      executed=node.Request("/world/default/remove",request,1000,response,service_result);
    } else {std::cout<<"ERROR invalid operation"<<std::endl;continue;}
    if(executed&&service_result&&response.data())std::cout<<"OK"<<std::endl;
    else std::cout<<"ERROR executed="<<executed<<" service="<<service_result<<" response="<<response.data()<<std::endl;
  }
}
