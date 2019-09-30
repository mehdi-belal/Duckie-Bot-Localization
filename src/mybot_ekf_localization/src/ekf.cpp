/**
 *  @file    ekf.cpp
 *  @author  Mehdi Belal
 *  @date    19/06/2018
 *  @version 2.0
 *
 *  @brief EKF Localization, using ICP
 *
 *  @section DESCRIPTION
 *  Nodo ROS per la localizzazione di un robot differenziale  attraverso sensori laser in ambiente noto.
 *  Questo sistema è basato sull'utilizzo del filtro di Kalaman esteso.
 *  Il filtro di Kalman, e la sua versione estesa sono delle approssimazioni del filtro di Bayes, fanno uso di
 *  una struttura parametrizzata in cui tutte le grandezze sono rappresntate come gaussiane.
 *
 *   - I vettori di stato sono rappresentati da grandezze gaussiane
 *   - Probabilità condizionata del modello di di stato è una grandezza gaussiana
 *   - Probabilità condizionata del modello di misura è una grandezza gaussiana
 *   - Incertezze sono gaussiane
 *
 *  In generale una grandezza gausiana che viene trasformata per mezzo di un sistema lineare è ancora una grandezza
 *  gaussiana, si può sfruttare la stessa propietà nel caso del'EKF linearizzando i modelli che trasformano le
 *  grandezze
 *
 *  Il sistema si basa su localizzazione a mappa nota, il confronto tra la mappa e le misurazioni del sensore laser
 *  viene effettuata mediante ICP (Iterative Closest Point, metodo iterativo per il confronto tra due set di punti),
 *  in modo da risolvere il problema delle corrispondenze e della misurazione senza un approccio esaustivo.
 *
 */

#include "ekf.hpp"

//---------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------
//------------------------------------  ##  COSTRUTTORE  ##   ---------------------------------------------
//---------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------

//  ----------------------------------
//  COSTRUTTORE
//  ----------------------------------
/**
*   @brief  EKFnode(): costruttore e inizializzazione del nodo
*
*   @param  ros::NodeHandle& nh: nodeHandle per la gestione del nodo
*   @param  double &spin_rate: rate del sistema
*/
ROSNode::ROSNode(const ros::NodeHandle& nh, const double & spin_rate):
  listener(new tf::TransformListener(ros::Duration(10.0))), //inizializzazione di parametri
  map_cloud(new pcl::PointCloud<point_type>()),
  laser_cloud(new pcl::PointCloud<point_type>()),
  input(2), //vettore degli ingressi, dimensione 2
  prediction(3),
  odom(3)
{
    std::cout << "\n\n" << std::endl;
    std::cout << "  ----------------------------------------------" << std::endl;
    std::cout << "  -------         Progetto Esame:       --------" << std::endl;
    std::cout << "  -------   INTELLIGENT MOBILE ROBOTS   --------" << std::endl;
    std::cout << "  ----                                        --" << std::endl;
    std::cout << "  ---- Nodo per il processo di localizzazione --" << std::endl;
    std::cout << "  ----                                        --" << std::endl;
    std::cout << "  ---- Studente: Mehdi Belal                  --" << std::endl;
    std::cout << "  ---- Coordinatore: Paolo Valigi             --" << std::endl;
    std::cout << "  ----------------------------------------------" << std::endl;
    std::cout << "  ----------------------------------------------" << std::endl;
    std::cout << "\n\n" << std::endl;

    node_handle.param<std::string>("base_frame_id",base_frame, "chassis");
    node_handle.param<std::string>("odom_frame_id",odom_frame, "odom");
    node_handle.param<std::string>("map_frame_id",map_frame, "map");
    node_handle.param<std::string>("laser_frame_id",laser_frame, "hokuyo");

    //ingresso u = [v  w]
    input  = 0.0;

    //posizione predetta
    prediction = 0.0;

    //vettore posizione reale del robot
    odom = 0.0;

    initKalmanFilter();

    double duration = (1.0/std::max(spin_rate,1.0));
    timer = node_handle.createTimer(ros::Duration(duration), &ROSNode::timer_callback, this);

    //Topic sottoscritti
    map_sub_ = node_handle.subscribe("map", 1, &ROSNode::map_callback, this);
    laser_sub = node_handle.subscribe("mybot/laser/scan", 1, &ROSNode::laser_callback, this);
    cmdvel_sub = node_handle.subscribe("mybot/cmd_vel", 1, &ROSNode::input_callback, this); //scrivere una callback per ottenere dati da cmdvel
    odom_sub = node_handle.subscribe("mybot/odom", 1, &ROSNode::odom_callback, this);

    //Topic pubblicati
    map_pub_ = node_handle.advertise<sensor_msgs::PointCloud2>("/map_pub",1);
    laser_pub_ = node_handle.advertise<sensor_msgs::PointCloud2>("/laser_pub",1);
    draw_position = node_handle.advertise<visualization_msgs::Marker>("/draw_position",1);
    position_error = node_handle.advertise<geometry_msgs::Vector3>("/position_error",1);
}



//---------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------
//------------------------------------  ##  METODI  ##   --------------------------------------------------
//---------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------

//  ----------------------------------
//  INIZIALIZZAZIONE EKF
//  ----------------------------------
/**
*   @brief  initKalmanFilter(): Inizializzazione del filtro di Kalman
*
*   @param  void
*   @return void
*/
void ROSNode::initKalmanFilter(){

    // -------------------------------------
    // -- 1. modello di stato non lineare --
    // -------------------------------------
    //
    //   X = f(Xt-1, U) + w
    //   w = (0, Q)

    BFL::ColumnVector system_noise_mean(3);
    system_noise_mean = 0.0;

    BFL::SymmetricMatrix Q(3);
    Q = 0.0;
    Q(1,1) = 1.0;
    Q(2,2) = 1.0;
    Q(3,3) = 1.0;

    //incertezza gaussiana
    BFL::Gaussian system_uncertainty(system_noise_mean, Q);

    //crea modello in funzione di:
    // -- modello non lineare di motion
    // -- incertezza gaussiana
    system_pdf = boost::shared_ptr<BFL::NonLinearAnalyticConditionalGaussianMobile> (new  BFL::NonLinearAnalyticConditionalGaussianMobile(system_uncertainty));
    system_model = boost::shared_ptr<BFL::AnalyticSystemModelGaussianUncertainty> (new BFL::AnalyticSystemModelGaussianUncertainty (system_pdf.get()));


    // --------------------------
    // -- 2. modello di misura --
    // --------------------------
    //
    //   z = H*x + v
    //   v = N(0. R)

    // measurement noise, scalare
    BFL::ColumnVector measurement_noise_mean(3);
    measurement_noise_mean = 0.0;

    BFL::SymmetricMatrix R(3);
    R(1,1) = 1.0;
    R(2,2) = 1.0;
    R(3,3) = 1.0;

    //incertezza gaussiana
    BFL::Gaussian measurement_uncertainty(measurement_noise_mean, R);

    // modello di misura lineare, matrice H
    BFL::Matrix H(3,3);
    H = 0.0;
    H(1,1) = 1.0;
    H(2,2) = 1.0;
    H(3,3) = 1.0;

    //crea modello in funzione di:
    // -- matrice H
    // -- incertezza gaussiana
    measurement_pdf = boost::shared_ptr<BFL::LinearAnalyticConditionalGaussian> (new BFL::LinearAnalyticConditionalGaussian(H, measurement_uncertainty));
    measurement_model = boost::shared_ptr<BFL::LinearAnalyticMeasurementModelGaussianUncertainty> (new BFL::LinearAnalyticMeasurementModelGaussianUncertainty(measurement_pdf.get()));

    // --------------------------------
    // -- 3.  Filtro di Kalman       --
    // --------------------------------
    // per inizializzare il filtro di Kalman è necessaria una prior
    BFL::ColumnVector prior_mean(3);
    prior_mean(1) = x_init;         //0.0
    prior_mean(2) = y_init;         //0.0
    prior_mean(3) = theta_init;     //0.0

    BFL::SymmetricMatrix prior_cov(3);
    prior_cov=0.0;
    prior_cov(1,1) = 1.0;
    prior_cov(2,2) = 1.0;
    prior_cov(3,3) = 1.0;
    BFL::Gaussian prior(prior_mean,prior_cov); //gaussiana

    //filtro di Kalman
    filter = boost::shared_ptr<BFL::ExtendedKalmanFilter> (new BFL::ExtendedKalmanFilter(&prior));

}//end initKalmanFilter


//  ----------------------------------
//  PREDICT STEP
//  ----------------------------------
/**
*   @brief  predictStep(): passo di predizione del filtro di Kalman, aggiornamento
*                      del sistema in funzione delle grandezze odometriche
*
*   @param  void
*   @return void
*/
void ROSNode::predictStep(){

    //system model contiene già la descrizione del modello cinematico e jacobiano, la descrizione
    //infatti è contenuta in NonLinearAnalyticConditionalGaussianMobile
    filter->Update(system_model.get(),input);

    BFL::Pdf<BFL::ColumnVector> *posterior = filter->PostGet();

}


//  ----------------------------------
//  CORRECT STEP
//  ----------------------------------
/**
*   @brief correctStep(): aggiornamento dello stato attraverso le misurazioni ottenute
*                         da laser
*
*   @param  void
*   @return void
*/
void ROSNode::correctStep(){

    // PA

    //per calcolo della innovazione
    Eigen::Matrix4f correction_transform = icp.getFinalTransformation();

    BFL::ColumnVector observation_mean(3);

    BFL::Pdf<BFL::ColumnVector> * posterior = filter->PostGet();
    double angle = correction_transform.block<3,3>(0,0).eulerAngles(0,1,2)(2) + posterior->ExpectedValueGet()(3);
    angleOverflowCorrect(angle); //restituisce valore in [0, 2pi]

    //covarianza innovazione
    BFL::SymmetricMatrix observation_noise(3);
    double fitness_score = icp.getFitnessScore(max_correspondence_distance);
    observation_noise = 0.0;
    observation_noise(1,1) = icp_score_scale*fitness_score;
    observation_noise(2,2) = icp_score_scale*fitness_score;
    observation_noise(3,3) = icp_score_scale*fitness_score;


    //vettore misurazioni
    observation_mean(1)=correction_transform(0,3);
    observation_mean(2)=correction_transform(1,3);
    observation_mean(3)=correction_transform.block<3,3>(0,0).eulerAngles(0,1,2)(2);

    //determina measurement model e aggiornamento
    measurement_pdf->AdditiveNoiseSigmaSet(observation_noise);
    filter->Update(measurement_model.get(),observation_mean);

    //in funzione della nuova posizione aggiorna i legami tra i sistemi di riferimento
    broadcast_tf(ros::Time::now());
}



//  ----------------------------------
//  DISEGNO RISULTATO
//  ----------------------------------
/**
*   @brief  drawCovariance(): disegna su un topic posizione e covarianza
*
*   @param void
*   @return void
*/
void ROSNode::drawCovariance(const Eigen::Matrix2f& covMatrix){

    BFL::Pdf<BFL::ColumnVector> *posterior = filter->PostGet();
    BFL::ColumnVector media = posterior->ExpectedValueGet();

    tf::Stamped<tf::Pose> odom_to_map;
    geometry_msgs::PoseStamped pose;

    try{                                                            //crea trasformazione con:
        tf::Transform tmp_tf(tf::createQuaternionFromYaw(media(3)),   // - matrice rotazione quaternioni
                             tf::Vector3(media(1), media(2), 0.0));   // - vettore traslazione

        tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(), time_, base_frame); //time = now
        listener->transformPose(odom_frame, tmp_tf_stamped, odom_to_map);
        tf::poseStampedTFToMsg(odom_to_map, pose);//trasforma da stamped a pose

    }catch(tf::TransformException &ex){
        ROS_ERROR("Errore cambiamento sistema di riferimento marker");
        ROS_ERROR("%s",ex.what());
        return;
    } //serve per ottenere la posizione nel sistema della mappa

    visualization_msgs::Marker tempMarker;
    tempMarker.pose.position.x = pose.pose.position.x;
    tempMarker.pose.position.y = pose.pose.position.y;

    std::cout << "-----------" << std::endl;
    std::cout << "-----------" << std::endl;

    prediction(1) = pose.pose.position.x;
    prediction(2) = pose.pose.position.y;
    prediction(3) = media(3);
    std::cout << "POSIZONE REALE: Odom Gazebo: (" << odom(1) << ", "
                                                  << odom(2) << ", "
                                                  << odom(3) << ")" << std::endl;

    std::cout << "POSIZIONE PREDETTA - media: (" << pose.pose.position.x << ", "
                                                 << pose.pose.position.y << ", "
                                                 << media(3) << ")" << std::endl;
    std::cout << "POSIZIONE PREDETTA - covarianza:" << posterior->CovarianceGet() << std::endl;


    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig(covMatrix);

    const Eigen::Vector2f& eigValues (eig.eigenvalues());
    const Eigen::Matrix2f& eigVectors (eig.eigenvectors());

    float angle = (atan2(eigVectors(1, 0), eigVectors(0, 0)));

    tempMarker.type = visualization_msgs::Marker::SPHERE;//in modo da avere sfera schiacciata come un disco

    double lengthMajor = sqrt(eigValues[0]);
    double lengthMinor = sqrt(eigValues[1]);

    tempMarker.scale.x = 3.0*lengthMajor;
    tempMarker.scale.y = 3.0*lengthMinor;
    tempMarker.scale.z = 0.001; //schiacciato

    tempMarker.color.a = 1.0;
    tempMarker.color.r = 0.0;
    tempMarker.color.g = 1.0;
    tempMarker.color.b = 0.0;

    tempMarker.pose.orientation.w = cos(angle*0.5);
    tempMarker.pose.orientation.z = sin(angle*0.5);

    tempMarker.header.frame_id = map_frame;
    tempMarker.id = 0;

    draw_position.publish(tempMarker); //pubblica nel topic
}


//  ----------------------------------
//  BROADCAST DELLA TRASFORMAZIONE
//  ----------------------------------
/**
*   @brief  broadcast(): broadcast delle trasformazioni che sono cambiate nel tempo
*
*   @param  ros::Time &broad_cast_time: istante di tempo di broadcast
*   @return void
*/
void ROSNode::broadcast_tf(const ros::Time & broad_cast_time) {

    BFL::Pdf<BFL::ColumnVector> * posterior = filter->PostGet();
    BFL::ColumnVector estimated_mean=posterior->ExpectedValueGet();

    try{                                                                        //crea trasformazione con:
        tf::Transform tmp_tf(tf::createQuaternionFromYaw(estimated_mean(3)),            // - matrice rotazione quaternioni
                             tf::Vector3(estimated_mean(1), estimated_mean(2), 0.0));   // - vettore traslazione

        //crea stamped per effettiare trasformazione, con tempo inserito da invocazione
        tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(), broad_cast_time, base_frame);

        listener->transformPose(odom_frame, tmp_tf_stamped, odom_to_map);

    }catch(tf::TransformException) {
        ROS_DEBUG("Errore cambiamento sistema di rifermento");
        return;
    }

    tf::Transform latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()), //rotazione
                                             tf::Point(odom_to_map.getOrigin()));       //traslazione


    //trasformazione con lo stamp temporale
    tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(), broad_cast_time, map_frame, odom_frame);
    transform_broadcaster.sendTransform(tmp_tf_stamped);

}

//  ----------------------------------
//  GESTIONE OVERFLOW ANGOLO
//  ----------------------------------
/**
*   @brief  angleOverflowCorrect(): Corregge overflow angolo se questo
*           supera il range [-pi, pi], si usa puntatore per cui restituisce void
*   @param void
*   @return void
*/
void ROSNode::angleOverflowCorrect(double &a) {
    while ((a) >  M_PI) a -= 2*M_PI;
    while ((a) < -M_PI) a += 2*M_PI;
}


//  ---------------------------------------------------------------------------------------------------------
//  ---------------------------------------------------------------------------------------------------------
//  ----------------------------------  ##  CALLBACKS  ##   -------------------------------------------------
//  ---------------------------------------------------------------------------------------------------------
//  ---------------------------------------------------------------------------------------------------------

//  ----------------------------------
//  CALLBACK RICHIAMATA DA ODOM
//  ----------------------------------
/**
*   @brief  odom_callback(): consente di visualizzare la posizione effettiva del sistema
*                            per verificare esattezza della predizione
*
*   @param  nav_msgs::Odometry& input_msg: Lettura dell'input fornito al sistema
*   @return void
*/
void ROSNode::odom_callback(const nav_msgs::Odometry& input_msg){

    double angle = tf::getYaw(input_msg.pose.pose.orientation); //angolo a partire da quaternioni
    odom(1) = input_msg.pose.pose.position.x;
    odom(2) = input_msg.pose.pose.position.y;
    odom(3) = angle;
}


//  ----------------------------------
//  CALLBACK RICHIAMATA DA LASER
//  ----------------------------------
/**
*   @brief  laser_callback()
*
*   @param  sensor_msgs::LaserScan::ConstPtr& msg: Lettura delle scansioni laser
*   @return void
*/
void ROSNode::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {

    if(map_state){ //callback eseguita quando si hanno a disposizione informazioni della mappa

      // Trasformazione tra il sistema di rifermento globale e il laser
      tf::StampedTransform laserToBaseTf;
      tf::StampedTransform baseDeltaTf;
      laser_stamp = msg->header.stamp;

      try{
          listener->waitForTransform(map_frame, laser_frame, msg->header.stamp, ros::Duration(0.1) );
          listener->lookupTransform(map_frame, laser_frame,  msg->header.stamp,  laserToBaseTf);

          listener->waitForTransform(base_frame, laser_stamp, base_frame, msg->header.stamp , odom_frame, ros::Duration(0.1) );
          listener->lookupTransform(base_frame, laser_stamp, base_frame, msg->header.stamp, odom_frame, baseDeltaTf);

      }catch(tf::TransformException& ex){
          //vogliamo che messaggi laser abbiano timestamp corretto, altrimenti si hanno problemi nel
          //cambiamento di rifermento e listener non funzionano
          laser_stamp = msg->header.stamp;
          ROS_WARN("Errore cambiamento sistema di rifemento laser");
          return;
      }

      // Trasforma laser in sensor_msgs::PointCloud
      sensor_msgs::PointCloud2 cloud_msg;
      //projector.transformLaserScanToPointCloud(map_frame, *msg, cloud_msg);
      projector.transformLaserScanToPointCloud(map_frame, *msg, cloud_msg, *listener);

      // Trasforma nel formato di pointcloud library
      pcl::PCLPointCloud2 pcl_cloud_msg;

      pcl_conversions::toPCL(cloud_msg, pcl_cloud_msg);
      pcl::fromPCLPointCloud2(pcl_cloud_msg, *laser_cloud);

      // Rimuovi NAN
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
      pcl::removeNaNFromPointCloud((*laser_cloud), *laser_cloud,inliers->indices);

      // Downsample, utilizza una VoxelGrid
      pcl::VoxelGrid<point_type> voxel_grid;
      voxel_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
      voxel_grid.setInputCloud (laser_cloud);
      voxel_grid.filter (*laser_cloud);

      //imposta lo stesso stamp per entrambe le pointcloud, in modo che in ICP i metodi
      //setInputSource e setInputTarget non abbiano problemi
      pcl_conversions::toPCL(msg->header.stamp, map_cloud->header.stamp);

      laser_state = true; //solo se laser true si applica icp
    }
}

//  ----------------------------------
//  CALLBACK RICHIAMATA DA MAPPA
//  ----------------------------------
/**
*   @brief  map_callback()
*
*   @param  nav_msgs::OccupancyGrid& map_msg Lettura del topic della mappa
*   @return void
*/
void ROSNode::map_callback(const nav_msgs::OccupancyGrid& map_msg) {

  //impostazione dei parametri della pointcloud della mappa attraverso i dati
  //della OccupancyGrid della mappa
  //map_ = new pcl::PointCloud<point_type>();
  map_cloud -> is_dense = false;
  map_cloud -> header.frame_id = map_msg.header.frame_id; //nel sistema di rifermento mappa
  double map_size_x = map_msg.info.width;
  double map_size_y = map_msg.info.height;
  double map_scale = map_msg.info.resolution;

  //Viene popolata la poinrcloud che rappresenta la mappa a partire dai dati
  //di nav_msgs::OccupancyGrid del topic map
  for(int i=0;i < map_size_x; ++i) { //Colonne
      for(int j=0;j < map_size_y; ++j) { //Righe
          if(map_msg.data[i+j*map_size_x] > 0.0) {
              point_type point;
              point.x = map_scale*i + map_msg.info.origin.position.x;
              point.y = map_scale*j + map_msg.info.origin.position.y;
              point.z = 0.0;

              map_cloud->push_back(point);
          }
      }
  }
  //downsampling con voxelgrid anche in questo caso, in modo da confrontare bene
  //le due pointcloud
  pcl::VoxelGrid<point_type> voxel_grid;
  voxel_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
  voxel_grid.setInputCloud (map_cloud);
  voxel_grid.filter (*map_cloud);

  map_state = true;

}


//-------------------------------------------------------------------
//CALLBACK RICHIAMATA DA TOPIC DEGLI INPUT
/**
 * @brief ROSNode::input_callback
 * @param input_msg
 */
void ROSNode::input_callback(const geometry_msgs::Twist& input_msg){
    //il sistema si aspetta di ricevere in ingresso solamente velocità lineare e velocità
    //angolare, le altre velocità, poichè si ha a che fare con un robot differential drive
    //2D non hanno significato in questo contesto.
    input(1) = input_msg.linear.x;
    input(2) = input_msg.angular.z;

   // if(input(1) != 0,0 || input(2) != 0,0){
        odom_state = true;
   // }

}


//  ----------------------------------
//  CALLBACK RICHIAMATA DAL TIMER
//  ----------------------------------s
/**
*   @brief  timercallback()
*
*   @param  ros::TimerEvent& e: esecuzione del sistema, all'aggiornamento del timer
*   @return void
*/
void ROSNode::timer_callback(const ros::TimerEvent& e){
  //1. Predict step
  if(odom_state){
    predictStep();
  }

  //2. Check delle callback per "forzare" esecuzione
  ros::spinOnce();

  //3. Calcolo ICP e determinazione della eventuale convergenza
  if(laser_state){
      //input e output
      icp.setInputSource(laser_cloud);
      icp.setInputTarget(map_cloud);

      //criteri
      icp.setTransformationEpsilon (icp_optimization_epsilon);
      icp.setMaxCorrespondenceDistance(max_correspondence_distance); //10

      //numero iterazioni
      icp.setMaximumIterations(max_iterations); //200 iterazioni

      //RANSAC per outliers
      icp.setRANSACIterations(ransac_iterations);
      icp.setRANSACOutlierRejectionThreshold(ransac_outlier_threshold);

      //output
      pcl::PointCloud<point_type> Final;
      icp.align(Final);
      Final.header.frame_id = map_frame;
      pcl_conversions::toPCL(time_, Final.header.stamp);

      if(icp.hasConverged()){
          //ROS_INFO("ICP converge correttamente -> measurement update");
          //4. Measurement update
          correctStep();

      }else{
          ROS_WARN("ICP non converge");
      }


      //5. Pubblicazione delle features e del marker di visualizzazione
      map_pub_.publish(map_cloud);
      laser_pub_.publish(laser_cloud);

      Eigen::Matrix2f covMatrix; //per la rappresentazione grafica
      BFL::Pdf<BFL::ColumnVector> *posterior = filter->PostGet();
      BFL::SymmetricMatrix estimated_cov=posterior->CovarianceGet();

      covMatrix(0,0)=estimated_cov(1,1);
      covMatrix(0,1)=estimated_cov(1,2);

      covMatrix(1,0)=estimated_cov(2,1);
      covMatrix(1,1)=estimated_cov(2,2);
      drawCovariance(covMatrix);

      //6. pubblicazione dell'errore di posizionamento
      //double angle = tf::getYaw(post.pose.orientation); //angolo a partire da quaternioni
      geometry_msgs::Vector3 err;
      err.x = (odom(1) - prediction(1))*3;
      err.y = (odom(2) - prediction(2))*3;
      err.z = (odom(3) - prediction(3));

      position_error.publish(err);
  }

}
