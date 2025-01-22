#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <GeographicLib/Geodesic.hpp>
#include <sdf/sdf.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  class GeoCoordinatesPlugin : public ModelPlugin
  {
  public:
    GeoCoordinatesPlugin() : ModelPlugin() {}

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      gzdbg << "GeoCoordinatesPlugin Load function called!" << std::endl;

      this->model = _model;
      this->world = _model->GetWorld();

      // Parâmetros de georreferência
      this->lat0 = 37.4275; // Exemplo: Stanford University
      this->lon0 = -122.1697;
      this->alt0 = 0.0;

      if (_sdf->HasElement("latitude_origin"))
      {
        this->lat0 = _sdf->Get<double>("latitude_origin");
      }
      if (_sdf->HasElement("longitude_origin"))
      {
        this->lon0 = _sdf->Get<double>("longitude_origin");
      }
      if (_sdf->HasElement("altitude_origin"))
      {
        this->alt0 = _sdf->Get<double>("altitude_origin");
      }

      // Inicializar o nó de transporte do Gazebo
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(this->world->Name());

      if (this->node)
        gzdbg << "Transport node initialized successfully." << std::endl;
      else
        gzerr << "Failed to initialize transport node." << std::endl;

      // Anunciar o tópico
      this->pub = this->node->Advertise<gazebo::msgs::Pose_V>("~/geo_coordinates");

      if (this->pub)
        gzdbg << "GeoCoordinatesPlugin topic advertised successfully." << std::endl;
      else
        gzerr << "Failed to advertise topic." << std::endl;

      // Conectar o evento de atualização do mundo
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&GeoCoordinatesPlugin::OnUpdate, this));
    }

    void OnUpdate()
    {
      gazebo::msgs::Pose_V msg;

      // Processar a solicitação e obter as coordenadas de todos os modelos
      for (auto model : this->world->Models())
      {
        // Processar apenas os links principais
        for (auto link : model->GetLinks())
        {
          std::string linkName = link->GetName();
          // Ignorar links que contêm "visual" ou "collision"
          if (linkName.find("visual") != std::string::npos || linkName.find("collision") != std::string::npos)
          {
            continue;
          }

          ignition::math::Pose3d pose = link->WorldPose();

          double x = pose.Pos().X();
          double y = pose.Pos().Y();
          double z = pose.Pos().Z();
          double roll = pose.Rot().Roll();
          double pitch = pose.Rot().Pitch();
          double yaw = pose.Rot().Yaw();

          GeographicLib::Geodesic geod = GeographicLib::Geodesic::WGS84();
          double lat, lon, azimuth;
          geod.Direct(this->lat0, this->lon0, atan2(y, x) * 180 / M_PI, sqrt(x * x + y * y), lat, lon);
          double alt = this->alt0 + z;

          gazebo::msgs::Pose *linkPose = msg.add_pose();
          linkPose->mutable_position()->set_x(lat);
          linkPose->mutable_position()->set_y(lon);
          linkPose->mutable_position()->set_z(alt);

          gazebo::msgs::Quaternion *orientation = linkPose->mutable_orientation();
          orientation->set_x(pose.Rot().X());
          orientation->set_y(pose.Rot().Y());
          orientation->set_z(pose.Rot().Z());
          orientation->set_w(pose.Rot().W());

          linkPose->set_name(model->GetName() + "::" + link->GetName());
        }
      }

      this->pub->Publish(msg);
      //gzdbg << "Published coordinates." << std::endl;
    }

  private:
    physics::ModelPtr model;
    physics::WorldPtr world;
    event::ConnectionPtr updateConnection;
    transport::NodePtr node;
    transport::PublisherPtr pub;

    double lat0, lon0, alt0;
  };

  GZ_REGISTER_MODEL_PLUGIN(GeoCoordinatesPlugin)
}

