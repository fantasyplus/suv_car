#include "nmea_parser.h"

#include <string>
#include <iostream>
#include <cmath>
#include <boost/xpressive/xpressive.hpp>
#include <boost/lexical_cast.hpp>
//$GPCHC,0,86.440,0.00,-0.95,0.71,-0.26,0.14,-0.16,-0.0122,-0.0163,0.9847,,,0.00,0.000,0.000,0.000,0.000,0,0,01,0,3*77
static auto gpchc = boost::xpressive::sregex::compile(R"--(^\$GPCHC,(\d+),([\d\.]+),([\d\.]+),([-\d\.]+),([-\d\.]+),[-\d\.]+,[-\d\.]+,[-\d\.]+,[-\d\.]+,[-\d\.]+,[-\d\.]+,([-\d\.]+),([-\d\.]+),([-\d\.]+),[-\d\.]+,[-\d\.]+,[-\d\.]+,([-\d\.]+),\d+,\d+,(\d+),\d+,\d+\*([0-9A-F]+)\r\n)--");

static auto gpgga = boost::xpressive::sregex::compile(R"--(^\$GPGGA,([\d\.]+),([\d\.]+),N,([\d\.]+),E,[0-5],\d+,[\d\.]+,([-\d\.]+),M,[-\d\.]+,M,\d*\*([0-9A-F]+)\r\n)--");

void reset_stamped_pose(stamped_pose &sp)
{
    sp.utc_time = 0;
    sp.lat = 0;
    sp.lon = 0;
    sp.alt = 0;
    sp.yaw = 0;
    sp.pitch = 0;
    sp.roll = 0;
    sp.velo = 0;
}

bool parse_nmea(const std::string &nmea_line, const std::string &nmea_head, std::string &valid_line, stamped_pose &sp)
{
    boost::xpressive::smatch what;
    bool is_valid_sentence = false;
    if ("GPGGA" == nmea_head)
    {
        is_valid_sentence = boost::xpressive::regex_match(nmea_line, what, gpgga);
        if (is_valid_sentence)
        {
            valid_line = what[0];
            sp.utc_time = boost::lexical_cast<double>(what[1]);
            sp.lat = boost::lexical_cast<double>(what[2]);
            double lad = std::floor(sp.lat / 100.0);
            sp.lat = (lad + (sp.lat - lad * 100) / 60.0) * M_PI / 180.0;
            sp.lon = boost::lexical_cast<double>(what[3]);
            double lod = std::floor(sp.lon / 100.0);
            sp.lon = (lod - (sp.lon - lod * 100) / 60.0) * M_PI / 180.0;
            sp.alt = boost::lexical_cast<double>(what[4]);
        }
    }
    else if ("GPCHC" == nmea_head)
    {
        is_valid_sentence = boost::xpressive::regex_match(nmea_line, what, gpchc);
        if (is_valid_sentence)
        {
            valid_line = what[0];
            //int week = boost::lexical_cast<int>(what[1]);
            //double second = boost::lexical_cast<double>(what[2]);
            //double week_secs = (week - 18 + 86400);
            //double day_secs = week_secs - (int(week_secs) % 86400);
            //int ss = int(day_secs);
            //double us = day_secs - ss;
            //int hh = int(ss / 3600);
            //ss = ss % 3600;
            //int mm = int(ss / 60);
            //ss = ss % 60;
            //sp.utc_time = (hh * 100 + mm) * 100 + ss + us;
            sp.yaw = boost::lexical_cast<double>(what[3]);
            sp.yaw = -sp.yaw * M_PI / 180.0 + M_PI / 2;
            sp.pitch = boost::lexical_cast<double>(what[4]);
            sp.pitch = -sp.pitch * M_PI / 180.0;
            sp.roll = boost::lexical_cast<double>(what[5]);
            sp.roll = sp.roll * M_PI / 180.0;
            sp.lat = boost::lexical_cast<double>(what[6]) * M_PI / 180.0;
            sp.lon = boost::lexical_cast<double>(what[7]) * M_PI / 180.0;
            sp.alt = boost::lexical_cast<double>(what[8]);
            sp.velo = boost::lexical_cast<double>(what[9]);
        }
    }
    else
    {
        std::cerr << "NMEA Parser: Not a valid head [" << nmea_head << "]" << std::endl;
    }

    return is_valid_sentence;
}

void blh2xyz(double lat, double lon, double alt, double lat_org, double lon_org, double &x, double &y, double &z)
{
    x = (lat - lat_org) * 6378137;
    y = (lon - lon_org) * 6378137 * std::cos(lat);
    z = alt;
    /*
    double PS;   //
    double PSo;  //
    double PDL;  //
    double Pt;   //
    double PN;   //
    double PW;   //

    double PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9;
    double PA, PB, PC, PD, PE, PF, PG, PH, PI;
    double Pe;   //
    double Pet;  //
    double Pnn;  //
    double AW, FW, Pmo;

    Pmo = 0.9999;

    //WGS84 Parameters
    AW = 6378137.0;            // Semimajor Axis
    FW = 1.0 / 298.257222101;  // 298.257223563 //Geometrical flattening

    Pe = static_cast<double>(std::sqrt(2.0 * FW - std::pow(FW, 2)));
    Pet = static_cast<double>(std::sqrt(std::pow(Pe, 2) / (1.0 - std::pow(Pe, 2))));
    PA = static_cast<double>(1.0 + 3.0 / 4.0 * std::pow(Pe, 2) + 45.0 / 64.0 * std::pow(Pe, 4) +
                             175.0 / 256.0 * std::pow(Pe, 6) + 11025.0 / 16384.0 * std::pow(Pe, 8) + 43659.0 / 65536.0 *
                                                                                                     std::pow(Pe, 10) +
                             693693.0 / 1048576.0 * std::pow(Pe, 12) + 19324305.0 / 29360128.0 * std::pow(Pe, 14) +
                             4927697775.0 / 7516192768.0 * std::pow(Pe, 16));

    PB = static_cast<double>(3.0 / 4.0 * std::pow(Pe, 2) + 15.0 / 16.0 * std::pow(Pe, 4) + 525.0 / 512.0 *
                                                                                           std::pow(Pe, 6) +
                             2205.0 / 2048.0 * std::pow(Pe, 8) + 72765.0 / 65536.0 * std::pow(Pe, 10) +
                             297297.0 / 262144.0 *
                             std::pow(Pe, 12) + 135270135.0 / 117440512.0 * std::pow(Pe, 14) +
                             547521975.0 / 469762048.0 * std::pow(Pe, 16));

    PC = static_cast<double>(15.0 / 64.0 * std::pow(Pe, 4) + 105.0 / 256.0 * std::pow(Pe, 6) + 2205.0 / 4096.0 *
                                                                                               std::pow(Pe, 8) +
                             10395.0 / 16384.0 * std::pow(Pe, 10) + 1486485.0 / 2097152.0 * std::pow(Pe, 12) +
                             45090045.0 / 58720256.0 * std::pow(Pe, 14) + 766530765.0 / 939524096.0 * std::pow(Pe, 16));

    PD = static_cast<double>(35.0 / 512.0 * std::pow(Pe, 6) + 315.0 / 2048.0 * std::pow(Pe, 8) + 31185.0 / 131072.0 *
                                                                                                 std::pow(Pe, 10) +
                             165165.0 / 524288.0 * std::pow(Pe, 12) + 45090045.0 / 117440512.0 * std::pow(Pe, 14) +
                             209053845.0 / 469762048.0 * std::pow(Pe, 16));

    PE = static_cast<double>(315.0 / 16384.0 * std::pow(Pe, 8) + 3465.0 / 65536.0 * std::pow(Pe, 10) +
                             99099.0 / 1048576.0 * std::pow(Pe, 12) + 4099095.0 / 29360128.0 * std::pow(Pe, 14) +
                             348423075.0 / 1879048192.0 *
                             std::pow(Pe, 16));

    PF = static_cast<double>(693.0 / 131072.0 * std::pow(Pe, 10) + 9009.0 / 524288.0 * std::pow(Pe, 12) +
                             4099095.0 / 117440512.0 * std::pow(Pe, 14) + 26801775.0 / 469762048.0 * std::pow(Pe, 16));

    PG = static_cast<double>(3003.0 / 2097152.0 * std::pow(Pe, 12) + 315315.0 / 58720256.0 * std::pow(Pe, 14) +
                             11486475.0 / 939524096.0 * std::pow(Pe, 16));

    PH = static_cast<double>(45045.0 / 117440512.0 * std::pow(Pe, 14) + 765765.0 / 469762048.0 * std::pow(Pe, 16));

    PI = static_cast<double>(765765.0 / 7516192768.0 * std::pow(Pe, 16));

    PB1 = static_cast<double>(AW) * (1.0 - std::pow(Pe, 2)) * PA;
    PB2 = static_cast<double>(AW) * (1.0 - std::pow(Pe, 2)) * PB / -2.0;
    PB3 = static_cast<double>(AW) * (1.0 - std::pow(Pe, 2)) * PC / 4.0;
    PB4 = static_cast<double>(AW) * (1.0 - std::pow(Pe, 2)) * PD / -6.0;
    PB5 = static_cast<double>(AW) * (1.0 - std::pow(Pe, 2)) * PE / 8.0;
    PB6 = static_cast<double>(AW) * (1.0 - std::pow(Pe, 2)) * PF / -10.0;
    PB7 = static_cast<double>(AW) * (1.0 - std::pow(Pe, 2)) * PG / 12.0;
    PB8 = static_cast<double>(AW) * (1.0 - std::pow(Pe, 2)) * PH / -14.0;
    PB9 = static_cast<double>(AW) * (1.0 - std::pow(Pe, 2)) * PI / 16.0;

    PS = static_cast<double>(PB1) * m_lat + PB2 * std::sin(2.0 * m_lat) + PB3 * std::sin(4.0 * m_lat) + PB4 * std::sin(6.0 * m_lat) +
         PB5 * std::sin(8.0 * m_lat) + PB6 * std::sin(10.0 * m_lat) + PB7 * std::sin(12.0 * m_lat) +
         PB8 * std::sin(14.0 * m_lat) + PB9 * std::sin(16.0 * m_lat);

    PSo = static_cast<double>(PB1) * m_PLato + PB2 * std::sin(2.0 * m_PLato) + PB3 * std::sin(4.0 * m_PLato) + PB4 * std::sin(6.0 * m_PLato) +
          PB5 * std::sin(8.0 * m_PLato) + PB6 * std::sin(10.0 * m_PLato) + PB7 * std::sin(12.0 * m_PLato) +
          PB8 * std::sin(14.0 * m_PLato) + PB9 * std::sin(16.0 * m_PLato);

    PDL = static_cast<double>(m_lon) - m_PLo;
    Pt = static_cast<double>(std::tan(m_lat));
    PW = static_cast<double>(std::sqrt(1.0 - std::pow(Pe, 2) * std::pow(std::sin(m_lat), 2)));
    PN = static_cast<double>(AW) / PW;
    Pnn = static_cast<double>(std::sqrt(std::pow(Pet, 2) * std::pow(std::cos(m_lat), 2)));

    m_x = static_cast<double>(
            ((PS - PSo) + (1.0 / 2.0) * PN * std::pow(std::cos(m_lat), 2.0) * Pt * std::pow(PDL, 2.0) +
             (1.0 / 24.0) * PN * std::pow(std::cos(m_lat), 4) * Pt *
             (5.0 - std::pow(Pt, 2) + 9.0 * std::pow(Pnn, 2) + 4.0 * std::pow(Pnn, 4)) * std::pow(PDL, 4) -
             (1.0 / 720.0) * PN * std::pow(std::cos(m_lat), 6) * Pt *
             (-61.0 + 58.0 * std::pow(Pt, 2) - std::pow(Pt, 4) - 270.0 * std::pow(Pnn, 2) + 330.0 *
                                                                                            std::pow(Pt, 2) *
                                                                                            std::pow(Pnn, 2)) *
             std::pow(PDL, 6) - (1.0 / 40320.0) * PN * std::pow(std::cos(m_lat), 8) * Pt *
                                (-1385.0 + 3111 * std::pow(Pt, 2) - 543 * std::pow(Pt, 4) + std::pow(Pt, 6)) *
                                std::pow(PDL, 8)) * Pmo);

    m_y = static_cast<double>((PN * std::cos(m_lat) * PDL -
                               1.0 / 6.0 * PN * std::pow(std::cos(m_lat), 3) *
                               (-1 + std::pow(Pt, 2) - std::pow(Pnn, 2)) * std::pow(PDL, 3) -
                               1.0 / 120.0 * PN * std::pow(std::cos(m_lat), 5) *
                               (-5.0 + 18.0 * std::pow(Pt, 2) - std::pow(Pt, 4) - 14.0 * std::pow(Pnn, 2) +
                                58.0 * std::pow(Pt, 2) *
                                std::pow(Pnn, 2)) * std::pow(PDL, 5) -
                               1.0 / 5040.0 * PN * std::pow(std::cos(m_lat), 7) *
                               (-61.0 + 479.0 * std::pow(Pt, 2) - 179.0 * std::pow(Pt, 4) + std::pow(Pt, 6)) *
                               std::pow(PDL, 7)) * Pmo);
    m_z = m_h;
    */
    return;
}