using LBDD.Dataplane;
using LBDD.Properties;
using LBDD.ui;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;

namespace LBDD.ExpermentsResults.Energy_consumptions
{
    /// <summary>
    /// Interaction logic for UISetParEnerConsum.xaml
    /// </summary>
    public partial class UISetParEnerConsum : Window
    {
        MainWindow _MainWindow;
        public UISetParEnerConsum(MainWindow __MainWindow_)
        {
            InitializeComponent();
            _MainWindow = __MainWindow_;

            try
            {


                com_queueTime.Items.Add("0.1");
                com_queueTime.Items.Add("0.2");
                com_queueTime.Items.Add("0.3");
                com_queueTime.Items.Add("0.4");
                com_queueTime.Items.Add("0.5");
                com_queueTime.Items.Add("0.6");
                com_queueTime.Items.Add("0.7");
                com_queueTime.Items.Add("0.8");
                com_queueTime.Items.Add("0.9");
                com_queueTime.Items.Add("1");
                com_queueTime.Items.Add("2");
                com_queueTime.Items.Add("3");
                com_queueTime.Items.Add("4");
                com_queueTime.Items.Add("5");


                

                com_queueTime.Text = Settings.Default.QueueTime.ToString();
               


                
            }
            catch
            {
                MessageBox.Show("Error!!!.");
            }

           
            Settings.Default.ShowRoutingPaths = false;
            Settings.Default.SaveRoutingLog = false;
            Settings.Default.ShowAnimation = false;
            Settings.Default.ShowRadar = false;

            for (int i = 60; i <= 1000; i = i + 60)
            {
                comb_simuTime.Items.Add(i);
               
            }
            comb_simuTime.Text = "300";

            comb_packet_rate.Items.Add("0.001");
            comb_packet_rate.Items.Add("0.01");
            comb_packet_rate.Items.Add("0.1");
            comb_packet_rate.Items.Add("0.5");
            for (int i = 1; i <= 5; i++)
            {
                comb_packet_rate.Items.Add(i);
            }

            comb_packet_rate.Text = "0.1";

            for(int i=5;i<=15;i++)
            {
                comb_startup.Items.Add(i);
            }
            comb_startup.Text = "10";

            for(int i=1;i<=5;i++)
            {
                comb_active.Items.Add(i);
                comb_sleep.Items.Add(i);
            }
            comb_active.Text = "1";
            comb_sleep.Text = "2";

           

            int conrange = 5;
            for (int i = 0; i <= conrange; i++)
            {
                if (i == conrange)
                {
                    double dc = Convert.ToDouble(i);
                   
                }
                else
                {
                    for (int j = 0; j <= 9; j++)
                    {
                        string str = i + "." + j;
                        double dc = Convert.ToDouble(str);
                       

                    }
                }
            }
        }


        private void btn_ok_Click(object sender, RoutedEventArgs e)
        {


            Settings.Default.DrawPacketsLines = Convert.ToBoolean(chk_drawrouts.IsChecked);
            Settings.Default.KeepLogs= Convert.ToBoolean(chk_save_logs.IsChecked);
            Settings.Default.StopeWhenFirstNodeDeid = Convert.ToBoolean(chk_stope_when_first_node_deis.IsChecked);
            Settings.Default.QueueTime = Convert.ToInt16(com_queueTime.Text);




            if (Settings.Default.StopeWhenFirstNodeDeid == false)
            {
                UInt32 stime = Convert.ToUInt32(comb_simuTime.Text);

                double packetRate = Convert.ToDouble(comb_packet_rate.Text);
                Settings.Default.StopSimlationWhen = stime;
                _MainWindow.RandomDeplayment();
                double numpackets = Convert.ToDouble(stime) / packetRate;
                _MainWindow.GenerateUplinkPacketsRandomly(Convert.ToInt32(numpackets));
                Settings.Default.PacketRate = packetRate;
                Settings.Default.Save();
            }
            else if (Settings.Default.StopeWhenFirstNodeDeid == true)
            {
                double packper = Convert.ToDouble(comb_packet_rate.Text);
                Settings.Default.StopSimlationWhen = 100000000;
                _MainWindow.RandomDeplayment();
                _MainWindow.SetPacketRateS1(packper);
                Settings.Default.Save();

            }

            Close();

        }

        private void comb_startup_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            object objval = comb_startup.SelectedItem as object;
            int va = Convert.ToInt16(objval);
            Settings.Default.MacStartUp = va;
        }

        private void comb_active_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            object objval = comb_active.SelectedItem as object;
            int va = Convert.ToInt16(objval);
            Settings.Default.ActivePeriod = va;
        }

        private void comb_sleep_SelectionChanged(object sender, SelectionChangedEventArgs e)
        {
            object objval = comb_sleep.SelectedItem as object;
            int va = Convert.ToInt16(objval);
            Settings.Default.SleepPeriod = va;
        }

        private void chk_stope_when_first_node_deis_Checked(object sender, RoutedEventArgs e)
        {
            comb_simuTime.IsEnabled = false;
        }

        private void chk_stope_when_first_node_deis_Unchecked(object sender, RoutedEventArgs e)
        {
            comb_simuTime.IsEnabled = true;
        }

      

        private void chk_drawrouts_Checked(object sender, RoutedEventArgs e)
        {
            Settings.Default.ShowRoutingPaths = true;
        }

        private void chk_drawrouts_Unchecked(object sender, RoutedEventArgs e)
        {
            Settings.Default.ShowRoutingPaths = false;
        }

        private void chk_save_logs_Checked(object sender, RoutedEventArgs e)
        {
            Settings.Default.SaveRoutingLog = true;
        }

        private void chk_save_logs_Unchecked(object sender, RoutedEventArgs e)
        {
            Settings.Default.SaveRoutingLog = false;
        }

        private void chek_show_radar_Checked(object sender, RoutedEventArgs e)
        {
            Settings.Default.ShowRadar = true;
        }

        private void chek_show_radar_Unchecked(object sender, RoutedEventArgs e)
        {
            Settings.Default.ShowRadar = false;
        }

        private void chek_animation_Checked(object sender, RoutedEventArgs e)
        {
            Settings.Default.ShowAnimation = true;
        }

        private void chek_animation_Unchecked(object sender, RoutedEventArgs e)
        {
            Settings.Default.ShowAnimation = false;
        }
    }
}
